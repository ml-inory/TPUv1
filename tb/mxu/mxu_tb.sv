//=============================================================================
// MXU (systolic array) self-checking testbench
// DUT: rtl/mxu/mxu.sv
//
// Intended micro-architecture (what this tb checks the wiring for):
//   * each row i feeds act_in[i] / act_in_valid through a delay_chain of
//     DEPTH=i before entering the array, so rows are skewed by i cycles
//   * each column j feeds psum_in[j] through a delay_chain of DEPTH=j
//   * inside the array activations flow LEFT->RIGHT (PE(i,j).act_out feeds
//     PE(i,j+1).act_in) and partial sums flow TOP->BOTTOM
//     (PE(i,j).psum_out feeds PE(i+1,j).psum_in)
//   * every PE is weight-stationary: it adds weight_reg*act_in into psum_in
//     whenever act_in_valid is high
//   * psum_out[j]      = PE(ROW-1,j).psum_out
//     psum_out_valid   = AND over j of PE(ROW-1,j).psum_out_valid
//
// The tb keeps a cycle-accurate golden model of exactly that structure and
// compares psum_out / psum_out_valid every cycle against the DUT, over a
// randomized activation stream with periodic weight loads and resets.
//
// Self-checking: prints PASS/FAIL, exits non-zero on any mismatch, dumps VCD.
// Waveform path: +wave=<file>  (default: mxu_tb.vcd)
//
// Every printed check label ends with the exact signal it compares.
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module mxu_tb;

    // Array size (deliberately non-square: catches ROW/COL mix-ups)
    localparam int ROW          = 3;
    localparam int COL          = 4;
    localparam int ACT_WIDTH    = 8;
    localparam int WEIGHT_WIDTH = 8;
    localparam int ACC_WIDTH    = 32;
    localparam int CLK_PERIOD   = 10; // ns
    localparam int PRIME_CYCLES = 30; // warm-up before comparisons start
    localparam int RUN_CYCLES   = 300;

    // ---- DUT signals --------------------------------------------------------
    logic clk, rst;

    logic                     load_weight;
    logic [WEIGHT_WIDTH-1:0]  weight_in [0:ROW-1][0:COL-1];

    logic                     act_in_valid;
    logic [ACT_WIDTH-1:0]     act_in    [0:ROW-1];

    logic                     psum_in_valid;
    logic [ACC_WIDTH-1:0]     psum_in   [0:COL-1];

    logic                     psum_out_valid;
    logic [ACC_WIDTH-1:0]     psum_out  [0:COL-1];

    // ---- DUT ----------------------------------------------------------------
    MXU #(
        .ROW(ROW), .COL(COL),
        .ACT_WIDTH(ACT_WIDTH), .WEIGHT_WIDTH(WEIGHT_WIDTH), .ACC_WIDTH(ACC_WIDTH)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .load_weight    (load_weight),
        .weight_in      (weight_in),
        .act_in_valid   (act_in_valid),
        .act_in         (act_in),
        .psum_in_valid  (psum_in_valid),
        .psum_in        (psum_in),
        .psum_out_valid (psum_out_valid),
        .psum_out       (psum_out)
    );

    // ---- Clock --------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- Scoreboard ---------------------------------------------------------
    int errors = 0;
    int checks = 0;
    bit cmp_en = 1'b0;   // comparisons only after the warm-up

    // =========================================================================
    //  Golden model state
    // =========================================================================
    // row act delay chains (DEPTH = i)
    int                      g_adcnt  [0:ROW-1];
    bit                      g_addone [0:ROW-1];
    logic [ACT_WIDTH-1:0]    g_adout  [0:ROW-1];
    // row valid delay chains (DEPTH = i)
    int                      g_vdcnt  [0:ROW-1];
    bit                      g_vddone [0:ROW-1];
    bit                      g_vdout  [0:ROW-1];
    // column psum delay chains (DEPTH = j)
    int                      g_pdcnt  [0:COL-1];
    bit                      g_pddone [0:COL-1];
    logic [ACC_WIDTH-1:0]    g_pdout  [0:COL-1];
    // PE registers
    logic [WEIGHT_WIDTH-1:0] g_w      [0:ROW-1][0:COL-1];
    logic [ACC_WIDTH-1:0]    g_psum   [0:ROW-1][0:COL-1];
    logic [ACT_WIDTH-1:0]    g_aout   [0:ROW-1][0:COL-1];
    bit                      g_pov    [0:ROW-1][0:COL-1];
    bit                      g_aov    [0:ROW-1][0:COL-1];
    // PE next state
    logic [WEIGHT_WIDTH-1:0] n_w      [0:ROW-1][0:COL-1];
    logic [ACC_WIDTH-1:0]    n_psum   [0:ROW-1][0:COL-1];
    logic [ACT_WIDTH-1:0]    n_aout   [0:ROW-1][0:COL-1];
    bit                      n_pov    [0:ROW-1][0:COL-1];
    bit                      n_aov    [0:ROW-1][0:COL-1];
    // golden outputs
    logic [ACC_WIDTH-1:0]    g_psum_out       [0:COL-1];
    bit                      g_psum_out_valid;

    // scratch used inside golden_step
    logic [ACT_WIDTH-1:0]             gm_act;
    bit                               gm_actv;
    logic [ACC_WIDTH-1:0]             gm_psum;
    logic signed [2*WEIGHT_WIDTH-1:0] gm_prod;
    logic signed [ACC_WIDTH-1:0]      gm_pext;

    // =========================================================================
    //  Golden model: one rising-edge worth of updates
    // =========================================================================
    task automatic golden_step();
        // ---- PE next state, from the CURRENT interconnect values -----------
        for (int i = 0; i < ROW; i++) begin
            for (int j = 0; j < COL; j++) begin
                gm_act  = (j == 0) ? g_adout[i] : g_aout[i][j-1];
                gm_actv = (j == 0) ? g_vdout[i] : g_aov[i][j-1];
                gm_psum = (i == 0) ? g_pdout[j] : g_psum[i-1][j];

                if (rst) begin
                    n_w[i][j]    = '0;
                    n_pov[i][j]  = 1'b0;
                    n_aov[i][j]  = 1'b0;
                    n_psum[i][j] = g_psum[i][j];
                    n_aout[i][j] = g_aout[i][j];
                end else if (load_weight) begin
                    n_w[i][j]    = weight_in[i][j];
                    n_pov[i][j]  = 1'b0;
                    n_aov[i][j]  = 1'b0;
                    n_psum[i][j] = g_psum[i][j];
                    n_aout[i][j] = g_aout[i][j];
                end else begin
                    if (gm_actv) begin
                        gm_prod = $signed(g_w[i][j]) * $signed(gm_act);
                        gm_pext = gm_prod;                       // sign-extend to ACC
                        n_psum[i][j] = $signed(gm_psum) + gm_pext;
                        n_aout[i][j] = gm_act;
                    end else begin
                        n_psum[i][j] = g_psum[i][j];             // hold
                        n_aout[i][j] = g_aout[i][j];             // hold
                    end
                    n_pov[i][j] = gm_actv;
                    n_aov[i][j] = gm_actv;
                end
            end
        end

        // ---- delay chains (PE state already sampled above) -----------------
        for (int i = 0; i < ROW; i++) begin
            if (rst) begin
                g_adcnt[i] = 0; g_addone[i] = 1'b0;
                g_vdcnt[i] = 0; g_vddone[i] = 1'b0;
            end else begin
                if (g_adcnt[i] < i) begin g_adcnt[i] = g_adcnt[i] + 1; g_addone[i] = 1'b0; end
                else                begin g_addone[i] = 1'b1; g_adout[i] = act_in[i]; end
                if (g_vdcnt[i] < i) begin g_vdcnt[i] = g_vdcnt[i] + 1; g_vddone[i] = 1'b0; end
                else                begin g_vddone[i] = 1'b1; g_vdout[i] = act_in_valid; end
            end
        end
        for (int j = 0; j < COL; j++) begin
            if (rst) begin
                g_pdcnt[j] = 0; g_pddone[j] = 1'b0;
            end else if (g_pdcnt[j] < j) begin
                g_pdcnt[j] = g_pdcnt[j] + 1; g_pddone[j] = 1'b0;
            end else begin
                g_pddone[j] = 1'b1; g_pdout[j] = psum_in[j];
            end
        end

        // ---- commit PE state and publish golden outputs --------------------
        g_psum_out_valid = 1'b1;
        for (int i = 0; i < ROW; i++) begin
            for (int j = 0; j < COL; j++) begin
                g_w[i][j]    = n_w[i][j];
                g_psum[i][j] = n_psum[i][j];
                g_aout[i][j] = n_aout[i][j];
                g_pov[i][j]  = n_pov[i][j];
                g_aov[i][j]  = n_aov[i][j];
            end
        end
        for (int j = 0; j < COL; j++) begin
            g_psum_out[j]    = n_psum[ROW-1][j];
            g_psum_out_valid = g_psum_out_valid & n_pov[ROW-1][j];
        end
    endtask

    // drive rst on the falling edge, advance the golden model with the same
    // inputs, sample the DUT one #1 after the rising edge, then compare
    task automatic step(input logic r);
        @(negedge clk);
        rst = r;
        golden_step();
        @(posedge clk); #1;
        if (cmp_en) compare_out();
    endtask

    // =========================================================================
    //  Checkers
    // =========================================================================
    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-38s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-38s = %b", name, got);
        end
    endtask

    // NOTE: psum_out is read through dut.psum_out[j] rather than the tb-level
    // array. Icarus Verilog does not propagate an unpacked-array *variable*
    // output port to the parent, so the tb-level copy would read X even when
    // the module itself is correct. Reading the module's own port works for a
    // variable port, a net port and a packed-array port alike.
    task automatic compare_out();
        for (int j = 0; j < COL; j++) begin
            checks++;
            if (dut.psum_out[j] !== g_psum_out[j]) begin
                errors++;
                $display("  [FAIL] psum_out[%0d] got=0x%08h exp=0x%08h", j, dut.psum_out[j], g_psum_out[j]);
            end
        end
        checks++;
        if (psum_out_valid !== g_psum_out_valid) begin
            errors++;
            $display("  [FAIL] psum_out_valid got=%b exp=%b", psum_out_valid, g_psum_out_valid);
        end
    endtask

    // =========================================================================
    //  Waveform dump / watchdog
    // =========================================================================
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "mxu_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, mxu_tb);
    end

    initial begin
        #(CLK_PERIOD * 100000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // =========================================================================
    //  Stimulus
    // =========================================================================
    initial begin
        // input defaults
        rst = 1'b0; load_weight = 1'b0;
        act_in_valid = 1'b0; psum_in_valid = 1'b0;
        for (int i = 0; i < ROW; i++) act_in[i]    = '0;
        for (int j = 0; j < COL; j++) psum_in[j]   = '0;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++) weight_in[i][j] = '0;

        // golden model defaults (X where the DUT registers are X)
        for (int i = 0; i < ROW; i++) begin
            g_adcnt[i]=0; g_addone[i]=1'b0; g_adout[i]='x;
            g_vdcnt[i]=0; g_vddone[i]=1'b0; g_vdout[i]=1'bx;
        end
        for (int j = 0; j < COL; j++) begin
            g_pdcnt[j]=0; g_pddone[j]=1'b0; g_pdout[j]='x;
        end
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++) begin
                g_w[i][j]='0; g_psum[i][j]='x; g_aout[i][j]='x;
                g_pov[i][j]=1'b0; g_aov[i][j]=1'b0;
            end

        $display("=================================================");
        $display(" MXU testbench  (ROW=%0d, COL=%0d, ACT=%0d, W=%0d, ACC=%0d)",
                 ROW, COL, ACT_WIDTH, WEIGHT_WIDTH, ACC_WIDTH);
        $display("=================================================");

        // ---- [1] Reset ------------------------------------------------------
        $display("[1] Reset");
        step(1'b1);
        check_bit("reset psum_out_valid", psum_out_valid, 1'b0);
        step(1'b1);

        // ---- [2] Load weights ----------------------------------------------
        $display("[2] Load weights");
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++) weight_in[i][j] = i*COL + j + 1;
        load_weight = 1'b1;
        step(1'b0);
        load_weight = 1'b0;
        check_bit("load psum_out_valid", psum_out_valid, 1'b0);

        // ---- [3] Warm-up with act_in_valid held high ------------------------
        // ensures every PE has accumulated so psum_out is defined before the
        // cycle-by-cycle comparison starts
        $display("[3] Warm-up (%0d cycles, act_in_valid=1)", PRIME_CYCLES);
        for (int c = 0; c < PRIME_CYCLES; c++) begin
            act_in_valid = 1'b1;
            for (int i = 0; i < ROW; i++) act_in[i]  = c*ROW + i;
            for (int j = 0; j < COL; j++) psum_in[j] = c*100 + j;
            step(1'b0);
        end
        check_bit("warm-up psum_out_valid", psum_out_valid, 1'b1);
        cmp_en = 1'b1;

        // ---- [4] Randomized stream vs golden model --------------------------
        $display("[4] Randomized stream vs golden model (%0d cycles)", RUN_CYCLES);
        for (int c = 0; c < RUN_CYCLES; c++) begin
            if (c % 61 == 0) begin
                // reload weights (must deassert psum_out_valid)
                for (int i = 0; i < ROW; i++)
                    for (int j = 0; j < COL; j++) weight_in[i][j] = $urandom;
                act_in_valid = 1'b0;
                load_weight  = 1'b1;
                step(1'b0);
                load_weight  = 1'b0;
            end else if (c % 97 == 0) begin
                // periodic reset
                act_in_valid = 1'b0;
                step(1'b1);
            end else begin
                act_in_valid = ($urandom_range(0, 3) != 0);   // ~75% valid
                psum_in_valid = $urandom_range(0, 1);
                for (int i = 0; i < ROW; i++) act_in[i]  = $urandom;
                for (int j = 0; j < COL; j++) psum_in[j] = $urandom;
                step(1'b0);
            end
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
            $fatal(1, "MXU testbench failed");
        end
    end

endmodule

`default_nettype wire
