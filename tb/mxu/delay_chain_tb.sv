//=============================================================================
// delay_chain self-checking testbench
// DUT: rtl/mxu/delay_chain.sv - DEPTH-stage shift register
//
// Contract implemented by the DUT:
//   * rst = 1   -> every stage cleared to 0 (so dout = 0 for DEPTH >= 1)
//   * rst = 0   -> chains[0] <= din, chains[s] <= chains[s-1],
//                  dout = chains[DEPTH-1]
//   * DEPTH = 0 -> dout = din (pure wire, rst has no effect on it)
//   * therefore dout(c) = din(c - DEPTH): a *constant* DEPTH-cycle delay for
//     a stream. This is what lets a systolic array skew its rows internally;
//     the older counter-based version only aligned the start of a stream
//     (after arming it was a plain 1-cycle register).
//
// Coverage: WIDTH=8 instances with DEPTH = 0/1/2/3/8 plus a WIDTH=1 DEPTH=2
// instance (the way MXU drives the act-valid line), each compared every cycle
// against a per-cycle reference model, plus directed checks:
//   [1] reset clears every stage
//   [2] step response: a new din reaches dout after DEPTH cycles
//       (DEPTH = 0 shows it in the same cycle, so the count is 1 there)
//   [3] randomized stream with periodic resets vs the reference model
//
// Self-checking: prints PASS/FAIL, exits non-zero on any mismatch, dumps VCD.
// Waveform path: +wave=<file>  (default delay_chain_tb.vcd)
//
// Every printed check label ends with the exact signal it compares.
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module delay_chain_tb;

    localparam int WIDTH      = 8;
    localparam int N          = 5;    // WIDTH-bit instances, DEPTH = 0,1,2,3,8
    localparam int DEPTH1     = 2;    // WIDTH=1 instance depth
    localparam int MAXD       = 8;    // deepest line
    localparam int CLK_PERIOD = 10;   // ns

    // ---- DUT signals --------------------------------------------------------
    logic clk, rst;

    logic [WIDTH-1:0] din  [N];
    logic [WIDTH-1:0] dout [N];

    logic din1, dout1;

    // ---- DUT instances ------------------------------------------------------
    delay_chain #(.WIDTH(WIDTH), .DEPTH(0)) u0 (.clk(clk), .rst(rst), .din(din[0]), .dout(dout[0]));
    delay_chain #(.WIDTH(WIDTH), .DEPTH(1)) u1 (.clk(clk), .rst(rst), .din(din[1]), .dout(dout[1]));
    delay_chain #(.WIDTH(WIDTH), .DEPTH(2)) u2 (.clk(clk), .rst(rst), .din(din[2]), .dout(dout[2]));
    delay_chain #(.WIDTH(WIDTH), .DEPTH(3)) u3 (.clk(clk), .rst(rst), .din(din[3]), .dout(dout[3]));
    delay_chain #(.WIDTH(WIDTH), .DEPTH(8)) u4 (.clk(clk), .rst(rst), .din(din[4]), .dout(dout[4]));

    delay_chain #(.WIDTH(1), .DEPTH(DEPTH1)) uv (.clk(clk), .rst(rst), .din(din1), .dout(dout1));

    // depth table, mirrors the instantiations above
    int DEPTHS [N];

    // ---- Clock --------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- Scoreboard ---------------------------------------------------------
    int errors = 0;
    int checks = 0;

    // ---- Reference model: one shift register per instance -------------------
    logic [WIDTH-1:0] pipe      [N][0:MAXD-1];   // stages, pipe[k][0] is newest
    logic [WIDTH-1:0] ref_dout  [N];
    logic [0:MAXD-1]  pipe1;
    logic             ref_dout1;

    int first_def [N];                            // step-response measurement
    int first_def1;

    // =========================================================================
    //  Reference model
    // =========================================================================
    task automatic model_step(input logic r);
        for (int k = 0; k < N; k++) begin
            if (r) begin
                for (int s = 0; s < DEPTHS[k]; s++) pipe[k][s] = '0;
            end else if (DEPTHS[k] > 0) begin
                for (int s = DEPTHS[k]-1; s > 0; s--) pipe[k][s] = pipe[k][s-1];
                pipe[k][0] = din[k];
            end
            // DEPTH=0 is a pure wire, so it tracks din even on a reset cycle
            ref_dout[k] = (DEPTHS[k] == 0) ? din[k] : pipe[k][DEPTHS[k]-1];
        end

        if (r) begin
            for (int s = 0; s < DEPTH1; s++) pipe1[s] = 1'b0;
        end else if (DEPTH1 > 0) begin
            for (int s = DEPTH1-1; s > 0; s--) pipe1[s] = pipe1[s-1];
            pipe1[0] = din1;
        end
        ref_dout1 = (DEPTH1 == 0) ? din1 : pipe1[DEPTH1-1];
    endtask

    // drive rst at the falling edge, advance the model, then sample after the
    // rising edge (where the DUT updates) and let NBA updates settle (#1)
    task automatic step(input logic r);
        @(negedge clk);
        rst = r;
        model_step(r);
        @(posedge clk); #1;
    endtask

    // =========================================================================
    //  Checkers
    // =========================================================================
    task automatic check_val(input string name,
                             input logic [WIDTH-1:0] got,
                             input logic [WIDTH-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-42s got=%0d (0x%02h)  exp=%0d (0x%02h)",
                     name, got, got, exp, exp);
        end else begin
            $display("  [PASS] %-42s = %0d (0x%02h)", name, got, got);
        end
    endtask

    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-42s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-42s = %b", name, got);
        end
    endtask

    task automatic check_int(input string name, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-42s got=%0d  exp=%0d", name, got, exp);
        end else begin
            $display("  [PASS] %-42s = %0d", name, got);
        end
    endtask

    // silent model comparison (only failures are printed)
    task automatic compare_dout(input int cyc);
        for (int k = 0; k < N; k++) begin
            checks++;
            if (dout[k] !== ref_dout[k]) begin
                errors++;
                $display("  [FAIL] u%0d.dout (DEPTH=%0d) got=0x%02h exp=0x%02h [cycle %0d]",
                         k, DEPTHS[k], dout[k], ref_dout[k], cyc);
            end
        end
        checks++;
        if (dout1 !== ref_dout1) begin
            errors++;
            $display("  [FAIL] uv.dout (WIDTH=1,DEPTH=%0d) got=%b exp=%b [cycle %0d]",
                     DEPTH1, dout1, ref_dout1, cyc);
        end
    endtask

    // =========================================================================
    //  Waveform dump / watchdog
    // =========================================================================
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "delay_chain_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, delay_chain_tb);
    end

    initial begin
        #(CLK_PERIOD * 20000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // =========================================================================
    //  Stimulus
    // =========================================================================
    initial begin
        // model / depth table init
        DEPTHS[0] = 0; DEPTHS[1] = 1; DEPTHS[2] = 2; DEPTHS[3] = 3; DEPTHS[4] = 8;
        for (int k = 0; k < N; k++) begin
            for (int s = 0; s < MAXD; s++) pipe[k][s] = 'x;
            ref_dout[k] = 'x;
            din[k]      = '0;
        end
        for (int s = 0; s < MAXD; s++) pipe1[s] = 1'bx;
        ref_dout1 = 1'bx;
        din1 = 1'b0;
        rst  = 1'b0;

        $display("=================================================");
        $display(" delay_chain testbench  (WIDTH=%0d, DEPTH=0/1/2/3/8 + WIDTH=1 DEPTH=%0d)",
                 WIDTH, DEPTH1);
        $display("=================================================");

        // ---- [1] Reset clears every stage -----------------------------------
        $display("[1] Reset clears every stage");
        step(1'b1);
        for (int k = 0; k < N; k++)
            check_val($sformatf("u%0d.dout after rst (DEPTH=%0d)", k, DEPTHS[k]),
                      dout[k], (DEPTHS[k] == 0) ? din[k] : '0);
        check_bit("uv.dout after rst (DEPTH=2)", dout1, 1'b0);

        // ---- [2] Step response: a new din reaches dout after DEPTH cycles ----
        // settle at 0 first (a value different from the step), then apply 0x5A;
        // the first cycle where dout shows 0x5A is the line latency.
        $display("[2] Step response: din 0 -> 0x5A reaches dout after DEPTH cycles");
        step(1'b1);
        for (int c = 0; c < 2; c++) begin
            for (int k = 0; k < N; k++) din[k] = '0;
            din1 = 1'b0;
            step(1'b0);
        end
        for (int k = 0; k < N; k++) first_def[k] = -1;
        first_def1 = -1;
        for (int c = 1; c <= MAXD + 4; c++) begin
            for (int k = 0; k < N; k++) din[k] = 8'h5A;
            din1 = 1'b1;
            step(1'b0);
            for (int k = 0; k < N; k++)
                if (first_def[k] < 0 && (dout[k] === din[k])) first_def[k] = c;
            if (first_def1 < 0 && (dout1 === din1)) first_def1 = c;
            compare_dout(c);
        end
        for (int k = 0; k < N; k++)
            check_int($sformatf("u%0d.dout step latency (DEPTH=%0d)", k, DEPTHS[k]),
                      first_def[k], (DEPTHS[k] == 0) ? 1 : DEPTHS[k]);
        check_int($sformatf("uv.dout step latency (DEPTH=%0d)", DEPTH1),
                  first_def1, (DEPTH1 == 0) ? 1 : DEPTH1);

        // ---- [3] Randomized stream + periodic reset vs reference model ------
        $display("[3] Randomized stream + periodic reset vs reference model (300 cycles)");
        for (int c = 0; c < 300; c++) begin
            if (c % 53 == 0) begin
                step(1'b1);                          // periodic reset
            end else begin
                for (int k = 0; k < N; k++) din[k] = $urandom;
                din1 = $urandom_range(0, 1);
                step(1'b0);
            end
            compare_dout(c);
        end

        // ---- Summary --------------------------------------------------------
        $display("=================================================");
        if (errors == 0) begin
            $display(" *** TEST PASSED ***  (%0d checks, 0 failures)", checks);
            $display("=================================================");
            $finish;
        end else begin
            $display(" *** TEST FAILED ***  (%0d/%0d checks failed)", errors, checks);
            $display("=================================================");
            $fatal(1, "delay_chain testbench failed");
        end
    end

endmodule

`default_nettype wire
