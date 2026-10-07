//=============================================================================
// PE (Processing Element) self-checking testbench
// DUT: rtl/mxu/pe.sv
//
// Contract implemented by the DUT (priority: rst > load_weight > accumulate):
//   * rst = 1            -> weight_reg <= 0, psum_out_valid <= 0,
//                           act_out_valid <= 0
//                           (psum_out / act_out are NOT reset -> they hold)
//   * load_weight = 1    -> weight_reg <= weight_in, psum_out_valid <= 0,
//                           act_out_valid <= 0
//                           (psum_out / act_out hold)
//   * act_in_valid = 1   -> psum_out <= psum_in + sign_ext(weight_reg*act_in),
//                           act_out  <= act_in,
//                           psum_out_valid <= 1, act_out_valid <= 1
//   * act_in_valid = 0   -> psum_out / act_out hold, both valids <= 0
//
//   weight/act are treated as SIGNED W-bit values, psum is SIGNED 32-bit.
//   `mult` is a combinational 2*W signed product of weight_reg and act_in.
//
// Self-checking: prints PASS/FAIL, exits non-zero on any mismatch, dumps VCD.
// Waveform path: +wave=<file>  (default: pe_tb.vcd)
//
// Every printed check label ends with the exact signal it compares. The
// registered weight is observed as dut.weight_reg so the "weight_reg" label
// is checked against the real register, not inferred from psum_out.
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module pe_tb;

    localparam int ACT_WIDTH    = 8;
    localparam int WEIGHT_WIDTH = 8;
    localparam int ACC_WIDTH    = 32;
    localparam int CLK_PERIOD   = 10; // ns

    // ---- DUT signals --------------------------------------------------------
    logic                     clk;
    logic                     rst;
    logic                     load_weight;
    logic [WEIGHT_WIDTH-1:0]  weight_in;
    logic                     act_in_valid;
    logic [ACT_WIDTH-1:0]     act_in;
    logic                     act_out_valid;
    logic [ACT_WIDTH-1:0]     act_out;
    logic [ACC_WIDTH-1:0]     psum_in;
    logic [ACC_WIDTH-1:0]     psum_out;
    logic                     psum_out_valid;

    // ---- DUT ----------------------------------------------------------------
    PE #(
        .ACT_WIDTH    (ACT_WIDTH),
        .WEIGHT_WIDTH (WEIGHT_WIDTH),
        .ACC_WIDTH    (ACC_WIDTH)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .load_weight    (load_weight),
        .weight_in      (weight_in),
        .act_in_valid   (act_in_valid),
        .act_in         (act_in),
        .act_out_valid  (act_out_valid),
        .act_out        (act_out),
        .psum_in        (psum_in),
        .psum_out       (psum_out),
        .psum_out_valid (psum_out_valid)
    );

    // ---- Clock --------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- Scoreboard ---------------------------------------------------------
    int errors = 0;
    int checks = 0;

    task automatic check32(input string name,
                           input logic [ACC_WIDTH-1:0] got,
                           input logic [ACC_WIDTH-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-32s got=%0d (0x%08h)  exp=%0d (0x%08h)",
                     name, $signed(got), got, $signed(exp), exp);
        end else begin
            $display("  [PASS] %-32s = %0d (0x%08h)", name, $signed(got), got);
        end
    endtask

    task automatic check8(input string name,
                          input logic [ACT_WIDTH-1:0] got,
                          input logic [ACT_WIDTH-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-32s got=%0d (0x%02h)  exp=%0d (0x%02h)",
                     name, $signed(got), got, $signed(exp), exp);
        end else begin
            $display("  [PASS] %-32s = %0d (0x%02h)", name, $signed(got), got);
        end
    endtask

    task automatic checkbit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-32s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-32s = %b", name, got);
        end
    endtask

    task automatic checkw(input string name,
                          input logic [WEIGHT_WIDTH-1:0] got,
                          input logic [WEIGHT_WIDTH-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-32s got=%0d (0x%02h)  exp=%0d (0x%02h)",
                     name, $signed(got), got, $signed(exp), exp);
        end else begin
            $display("  [PASS] %-32s = %0d (0x%02h)", name, $signed(got), got);
        end
    endtask

    // Drive inputs on the falling edge so they are stable, then wait for the
    // rising edge where the PE samples them and settle NBA updates (#1).
    task automatic tick(input logic r,
                        input logic lw,
                        input logic [WEIGHT_WIDTH-1:0] wi,
                        input logic av,
                        input logic [ACT_WIDTH-1:0]    ai,
                        input logic [ACC_WIDTH-1:0]    pi);
        @(negedge clk);
        rst = r; load_weight = lw; weight_in = wi;
        act_in_valid = av; act_in = ai; psum_in = pi;
        @(posedge clk); #1;
    endtask

    // ---- Waveform dump ------------------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "pe_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, pe_tb);
    end

    // ---- Watchdog -----------------------------------------------------------
    initial begin
        #(CLK_PERIOD * 20000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- Stimulus -----------------------------------------------------------
    initial begin
        int i;

        $display("===============================================");
        $display(" PE testbench  (ACT=%0d WEIGHT=%0d ACC=%0d)",
                 ACT_WIDTH, WEIGHT_WIDTH, ACC_WIDTH);
        $display("===============================================");

        // Idle
        rst = 1'b0; load_weight = 1'b0; weight_in = '0;
        act_in_valid = 1'b0; act_in = '0; psum_in = '0;
        tick(1'b0, 1'b0, '0, 1'b0, '0, '0);

        // ---- 1. Reset clears control state ----------------------------------
        // psum_out / act_out are uninitialised (X) here and are intentionally
        // left untouched by reset, so only the control outputs are checked.
        $display("[1] Reset");
        tick(1'b1, 1'b0, '0, 1'b0, '0, '0);
        checkbit("reset  psum_out_valid", psum_out_valid, 1'b0);
        checkbit("reset  act_out_valid",  act_out_valid,  1'b0);

        // ---- 2. Load weight, then accumulate --------------------------------
        $display("[2] Load w=3, accumulate act=4,5");
        tick(1'b0, 1'b1, 8'sd3, 1'b0, '0, '0);           // load
        checkbit("load1  psum_out_valid", psum_out_valid, 1'b0);
        checkbit("load1  act_out_valid",  act_out_valid,  1'b0);
        checkw  ("load1  weight_reg",     dut.weight_reg,  8'sd3);
        tick(1'b0, 1'b0, '0, 1'b1, 8'sd4, 32'sd0);       // 0 + 3*4
        check32 ("acc1   psum_out",       psum_out,       32'sd12);
        check8  ("acc1   act_out",        act_out,        8'sd4);
        checkbit("acc1   psum_out_valid", psum_out_valid, 1'b1);
        checkbit("acc1   act_out_valid",  act_out_valid,  1'b1);
        tick(1'b0, 1'b0, '0, 1'b1, 8'sd5, 32'sd12);      // 12 + 3*5
        check32 ("acc2   psum_out",       psum_out,       32'sd27);
        check8  ("acc2   act_out",        act_out,        8'sd5);

        // ---- 3. act_in_valid=0 -> hold, valid deasserts ---------------------
        $display("[3] act_in_valid=0 holds psum_out/act_out");
        tick(1'b0, 1'b0, '0, 1'b0, 8'sd99, 32'sd27);
        check32 ("hold   psum_out",       psum_out,       32'sd27);
        check8  ("hold   act_out",        act_out,        8'sd5);
        checkbit("hold   psum_out_valid", psum_out_valid, 1'b0);
        checkbit("hold   act_out_valid",  act_out_valid,  1'b0);

        // ---- 4. load_weight has priority over act_in_valid ------------------
        $display("[4] load_weight beats act_in_valid");
        tick(1'b0, 1'b1, 8'sd7, 1'b1, 8'sd9, 32'sd111);  // load wins
        check32 ("load2  psum_out holds", psum_out,       32'sd27);
        check8  ("load2  act_out holds",  act_out,        8'sd5);
        checkbit("load2  psum_out_valid", psum_out_valid, 1'b0);
        checkbit("load2  act_out_valid",  act_out_valid,  1'b0);
        checkw  ("load2  weight_reg",     dut.weight_reg,  8'sd7);
        tick(1'b0, 1'b0, '0, 1'b1, 8'sd10, 32'sd0);      // 0 + 7*10
        check32 ("acc3   psum_out",       psum_out,       32'sd70);
        checkbit("acc3   psum_out_valid", psum_out_valid, 1'b1);
        checkbit("acc3   act_out_valid",  act_out_valid,  1'b1);

        // ---- 5. Signed operands --------------------------------------------
        $display("[5] Negative weight / activation");
        tick(1'b0, 1'b1, -8'sd2, 1'b0, '0, '0);          // w = -2
        checkw  ("neg1   weight_reg",     dut.weight_reg,  -8'sd2);
        tick(1'b0, 1'b0, '0, 1'b1, 8'sd10, 32'sd27);     // 27 + (-2*10)
        check32 ("neg1   psum_out",       psum_out,       32'sd7);
        tick(1'b0, 1'b0, '0, 1'b1, -8'sd3, 32'sd7);      // 7 + (-2*-3)
        check32 ("neg2   psum_out",       psum_out,       32'sd13);

        // ---- 6. Extreme products -------------------------------------------
        $display("[6] Extreme products");
        tick(1'b0, 1'b1, 8'sd127, 1'b0, '0, '0);
        checkw  ("max_pos weight_reg",    dut.weight_reg,  8'sd127);
        tick(1'b0, 1'b0, '0, 1'b1, 8'sd127, 32'sd0);     // 127*127
        check32 ("max_pos psum_out",      psum_out,       32'sd16129);
        tick(1'b0, 1'b1, -8'sd128, 1'b0, '0, '0);
        checkw  ("max_neg weight_reg",    dut.weight_reg,  -8'sd128);
        tick(1'b0, 1'b0, '0, 1'b1, -8'sd128, 32'sd0);    // (-128)*(-128)
        check32 ("max_neg psum_out",      psum_out,       32'sd16384);

        // ---- 7. Large psum_in is not truncated -----------------------------
        $display("[7] Large psum_in chain");
        tick(1'b0, 1'b1, 8'sd1, 1'b0, '0, '0);
        checkw  ("chain  weight_reg",     dut.weight_reg,  8'sd1);
        tick(1'b0, 1'b0, '0, 1'b1, 8'sd1, 32'sd1_000_000);
        check32 ("chain  psum_out",       psum_out,       32'sd1_000_001);
        check8  ("chain  act_out",        act_out,        8'sd1);

        // ---- 8. Reset clears weight_reg but NOT psum_out/act_out ------------
        $display("[8] Reset clears weight, holds datapath (current RTL)");
        tick(1'b1, 1'b0, '0, 1'b0, '0, '0);
        check32 ("rst2   psum_out holds", psum_out,       32'sd1_000_001);
        check8  ("rst2   act_out holds",  act_out,        8'sd1);
        checkbit("rst2   psum_out_valid", psum_out_valid, 1'b0);
        checkbit("rst2   act_out_valid",  act_out_valid,  1'b0);
        checkw  ("rst2   weight_reg",     dut.weight_reg,  8'sd0);
        // weight_reg == 0 -> next accumulate adds nothing
        tick(1'b0, 1'b0, '0, 1'b1, 8'sd5, 32'sd0);
        check32 ("rst2   psum_out",       psum_out,       32'sd0);

        // ---- 9. Reset has priority over load_weight -------------------------
        $display("[9] Reset beats load_weight");
        tick(1'b1, 1'b1, 8'sd123, 1'b1, 8'sd45, 32'sd999); // reset wins
        check32 ("rst3   psum_out holds", psum_out,       32'sd0);
        check8  ("rst3   act_out holds",  act_out,        8'sd5);
        checkbit("rst3   psum_out_valid", psum_out_valid, 1'b0);
        checkbit("rst3   act_out_valid",  act_out_valid,  1'b0);
        checkw  ("rst3   weight_reg",     dut.weight_reg,  8'sd0);
        tick(1'b0, 1'b0, '0, 1'b1, 8'sd1, 32'sd0);       // weight still 0
        check32 ("rst3   psum_out",       psum_out,       32'sd0);

        // ---- 10. Randomized cross-check vs reference model ------------------
        $display("[10] Randomized cross-check (300 cycles)");
        begin
            logic signed [WEIGHT_WIDTH-1:0] ref_weight;
            logic        [ACC_WIDTH-1:0]    ref_psum;
            logic        [ACT_WIDTH-1:0]    ref_act_out;
            logic                           ref_valid;
            logic                           ref_act_valid;

            logic                           lw, av, lr;
            logic        [WEIGHT_WIDTH-1:0] wi;
            logic        [ACT_WIDTH-1:0]    ai;
            logic        [ACC_WIDTH-1:0]    pi;

            // Clean reset, then one known accumulate so psum_out/act_out are
            // defined before the model starts comparing.
            tick(1'b1, 1'b0, '0, 1'b0, '0, '0);
            ref_weight  = '0;
            ref_valid   = 1'b0;
            ref_act_valid = 1'b0;
            tick(1'b0, 1'b0, '0, 1'b1, 8'sd0, 32'sd0); // w=0 -> psum_out=0
            ref_psum    = 32'sd0;
            ref_act_out = 8'sd0;

            for (i = 0; i < 300; i++) begin
                lr = ($urandom_range(0, 99) < 5);  // ~5% reset
                lw = (!$urandom_range(0, 4));       // ~20% load
                av = ($urandom_range(0, 9) < 8);    // ~80% valid act
                wi = $urandom;
                ai = $urandom;
                pi = $urandom;

                // Reference model: matches RTL priority rst > load > accumulate
                if (lr) begin
                    ref_weight = '0;
                    ref_valid  = 1'b0;
                    ref_act_valid = 1'b0;
                end else if (lw) begin
                    ref_weight = wi;
                    ref_valid  = 1'b0;
                    ref_act_valid = 1'b0;
                end else begin
                    ref_valid = av;
                    ref_act_valid = av;
                    if (av) begin
                        ref_psum    = pi + ACC_WIDTH'($signed(ref_weight) * $signed(ai));
                        ref_act_out = ai;
                    end
                end

                tick(lr, lw, wi, av, ai, pi);
                check32 ($sformatf("rand[%0d] psum_out", i), psum_out,       ref_psum);
                check8  ($sformatf("rand[%0d] act_out",  i), act_out,        ref_act_out);
                checkbit($sformatf("rand[%0d] psum_out_valid", i), psum_out_valid, ref_valid);
                checkbit($sformatf("rand[%0d] act_out_valid",  i), act_out_valid,  ref_act_valid);
            end
        end

        // ---- Summary --------------------------------------------------------
        $display("===============================================");
        if (errors == 0) begin
            $display(" *** TEST PASSED ***  (%0d checks, 0 failures)", checks);
            $display("===============================================");
            $finish;
        end else begin
            $display(" *** TEST FAILED ***  (%0d/%0d checks failed)",
                     errors, checks);
            $display("===============================================");
            $fatal(1, "PE testbench failed");
        end
    end

endmodule

`default_nettype wire
