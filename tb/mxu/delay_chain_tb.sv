//=============================================================================
// delay_chain self-checking testbench
// DUT: rtl/mxu/delay_chain.sv
//
// Contract implemented by the DUT:
//   * rst = 1  -> counter <= 0, delay_done <= 0   (dout is NOT reset -> holds)
//   * rst = 0  -> while counter <  DEPTH : counter++, delay_done <= 0, dout holds
//                 once  counter >= DEPTH : delay_done <= 1, dout <= din
//
//   Effect: after reset is released, dout holds its previous value (undefined
//   at power-up) for DEPTH cycles and captures din starting on edge DEPTH+1;
//   from then on dout follows din with a fixed 1-cycle latency. The module
//   aligns the *start* of a stream - it is NOT a DEPTH-deep shift register.
//
// Coverage: five WIDTH=8 instances (DEPTH = 0/1/2/3/8) plus one WIDTH=1
// instance (DEPTH=2, the way MXU uses it for the valid line), checked against
// a per-cycle reference model with directed tests, arming-latency checks,
// reset-hold checks and a randomized stream.
//
// Self-checking: prints PASS/FAIL, exits non-zero on any mismatch, dumps VCD.
// Waveform path: +wave=<file>  (default: delay_chain_tb.vcd)
//
// Every printed check label ends with the exact signal it compares.
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module delay_chain_tb;

    localparam int WIDTH      = 8;
    localparam int N          = 5;    // WIDTH-bit instances, DEPTH = 0,1,2,3,8
    localparam int DEPTH1     = 2;    // WIDTH=1 instance depth
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

    // ---- Reference model state ----------------------------------------------
    int   ref_counter [N];
    logic ref_done    [N];
    logic [WIDTH-1:0] ref_dout [N];

    int   ref_counter1;
    logic ref_done1;
    logic ref_dout1;

    // scratch used by the arming-latency phase
    int first_def  [N];
    int first_def1;

    // =========================================================================
    //  Reference model
    // =========================================================================
    task automatic model_step(input logic r);
        if (r) begin
            for (int i = 0; i < N; i++) begin
                ref_counter[i] = 0;
                ref_done[i]    = 1'b0;
            end
            ref_counter1 = 0;
            ref_done1    = 1'b0;
        end else begin
            for (int i = 0; i < N; i++) begin
                if (ref_counter[i] < DEPTHS[i]) begin
                    ref_counter[i] = ref_counter[i] + 1;
                    ref_done[i]    = 1'b0;
                end else begin
                    ref_done[i]    = 1'b1;
                    ref_dout[i]    = din[i];
                end
            end
            if (ref_counter1 < DEPTH1) begin
                ref_counter1 = ref_counter1 + 1;
                ref_done1    = 1'b0;
            end else begin
                ref_done1    = 1'b1;
                ref_dout1    = din1;
            end
        end
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
            $display("  [FAIL] %-40s got=%0d (0x%02h)  exp=%0d (0x%02h)",
                     name, got, got, exp, exp);
        end else begin
            $display("  [PASS] %-40s = %0d (0x%02h)", name, got, got);
        end
    endtask

    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-40s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-40s = %b", name, got);
        end
    endtask

    task automatic check_int(input string name, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-40s got=%0d  exp=%0d", name, got, exp);
        end else begin
            $display("  [PASS] %-40s = %0d", name, got);
        end
    endtask

    // silent model comparison (only failures are printed)
    task automatic compare_dout(input int cyc);
        for (int i = 0; i < N; i++) begin
            checks++;
            if (dout[i] !== ref_dout[i]) begin
                errors++;
                $display("  [FAIL] u%0d.dout (DEPTH=%0d) got=0x%02h exp=0x%02h [cycle %0d]",
                         i, DEPTHS[i], dout[i], ref_dout[i], cyc);
            end
        end
        checks++;
        if (dout1 !== ref_dout1) begin
            errors++;
            $display("  [FAIL] uv.dout (WIDTH=1,DEPTH=%0d) got=%b exp=%b [cycle %0d]",
                     DEPTH1, dout1, ref_dout1, cyc);
        end
    endtask

    task automatic chk_counter(input string name, input logic [31:0] got,
                               input logic [31:0] exp, input int cyc);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-20s got=%0d exp=%0d [cycle %0d]", name, got, exp, cyc);
        end
    endtask

    task automatic chk_flag(input string name, input logic got,
                            input logic exp, input int cyc);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-20s got=%b exp=%b [cycle %0d]", name, got, exp, cyc);
        end
    endtask

    task automatic compare_internal(input int cyc);
        chk_counter("u0.counter",    u0.counter, ref_counter[0], cyc);
        chk_counter("u1.counter",    u1.counter, ref_counter[1], cyc);
        chk_counter("u2.counter",    u2.counter, ref_counter[2], cyc);
        chk_counter("u3.counter",    u3.counter, ref_counter[3], cyc);
        chk_counter("u4.counter",    u4.counter, ref_counter[4], cyc);
        chk_counter("uv.counter",    uv.counter, ref_counter1,   cyc);
        chk_flag("u0.delay_done",    u0.delay_done, ref_done[0], cyc);
        chk_flag("u1.delay_done",    u1.delay_done, ref_done[1], cyc);
        chk_flag("u2.delay_done",    u2.delay_done, ref_done[2], cyc);
        chk_flag("u3.delay_done",    u3.delay_done, ref_done[3], cyc);
        chk_flag("u4.delay_done",    u4.delay_done, ref_done[4], cyc);
        chk_flag("uv.delay_done",    uv.delay_done, ref_done1,   cyc);
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
        for (int i = 0; i < N; i++) begin
            ref_counter[i] = 0;
            ref_done[i]    = 1'b0;
            ref_dout[i]    = 'x;
            din[i]         = '0;
        end
        ref_counter1 = 0; ref_done1 = 1'b0; ref_dout1 = 1'bx;
        din1 = 1'b0;
        rst  = 1'b0;

        $display("=================================================");
        $display(" delay_chain testbench  (WIDTH=%0d, DEPTH=0/1/2/3/8 + WIDTH=1 DEPTH=%0d)",
                 WIDTH, DEPTH1);
        $display("=================================================");

        // ---- [1] Reset ------------------------------------------------------
        $display("[1] Reset");
        step(1'b1);
        check_int("u3.counter",     u3.counter,     0);
        check_bit("u3.delay_done",  u3.delay_done,  1'b0);
        check_int("uv.counter",     uv.counter,     0);
        check_bit("uv.delay_done",  uv.delay_done,  1'b0);

        // ---- [2] Prime: arm every instance ----------------------------------
        // dout is only defined once an instance has been armed, so run past the
        // deepest DEPTH (8) before comparing dout against the model.
        $display("[2] Arm all instances (u* din = 0x10..0x14, uv din = 1)");
        for (int c = 1; c <= 11; c++) begin
            for (int i = 0; i < N; i++) din[i] = 8'h10 + i;
            din1 = 1'b1;
            step(1'b0);
            compare_internal(c);
        end
        for (int i = 0; i < N; i++)
            check_val($sformatf("u%0d.dout armed value", i), dout[i], 8'h10 + i);
        check_bit("uv.dout armed value", dout1, 1'b1);
        check_bit("u4.delay_done armed", u4.delay_done, 1'b1);

        // ---- [3] Reset holds dout, clears counter/delay_done ----------------
        $display("[3] Reset holds dout, clears counter/delay_done");
        step(1'b1);
        for (int i = 0; i < N; i++)
            check_val($sformatf("u%0d.dout held on rst", i), dout[i], 8'h10 + i);
        check_bit("uv.dout held on rst", dout1, 1'b1);
        check_int("u4.counter",    u4.counter,    0);
        check_bit("u4.delay_done", u4.delay_done, 1'b0);

        // ---- [4] Arming latency: dout starts tracking on edge DEPTH+1 -------
        // din changes every cycle and never equals the value held through the
        // arming window, so the first cycle where dout == din is exactly the
        // cycle the instance starts passing data through.
        $display("[4] Arming latency: dout starts tracking on edge DEPTH+1");
        for (int i = 0; i < N; i++) first_def[i] = -1;
        first_def1 = -1;
        for (int c = 1; c <= 11; c++) begin
            for (int i = 0; i < N; i++) din[i] = 8'h80 + c;
            din1 = 1'b0;                                     // differs from held 1
            step(1'b0);
            for (int i = 0; i < N; i++)
                if (first_def[i] < 0 && (dout[i] === din[i])) first_def[i] = c;
            if (first_def1 < 0 && (dout1 === din1)) first_def1 = c;
            compare_dout(c);
            compare_internal(c);
        end
        for (int i = 0; i < N; i++)
            check_int($sformatf("u%0d.dout first_tracking_cycle (DEPTH=%0d)", i, DEPTHS[i]),
                      first_def[i], DEPTHS[i] + 1);
        check_int($sformatf("uv.dout first_tracking_cycle (DEPTH=%0d)", DEPTH1),
                  first_def1, DEPTH1 + 1);

        // ---- [5] Randomized stream + periodic reset vs reference model ------
        $display("[5] Randomized stream + periodic reset vs reference model (300 cycles)");
        for (int c = 0; c < 300; c++) begin
            if (c % 53 == 0) begin
                step(1'b1);                          // periodic reset
            end else begin
                for (int i = 0; i < N; i++) din[i] = $urandom;
                din1 = $urandom_range(0, 1);
                step(1'b0);
            end
            compare_dout(c);
            compare_internal(c);
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
