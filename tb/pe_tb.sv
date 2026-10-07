//=============================================================================
// PE (Processing Element) self-checking testbench
//
// Intent / DUT contract (mirrors rtl/pe.sv):
//   * rst = 1          -> w_s, psum_out, valid cleared
//   * rst = 0, load = 1 -> w_s <= w, valid <= 0, psum_out holds
//   * rst = 0, load = 0 -> psum_out <= psum_in + sign_ext(w_s * act),
//                          valid <= 1
//   w_s and act are treated as SIGNED 8-bit values, psum is SIGNED 32-bit.
//
// Reports PASS/FAIL, exits non-zero on any mismatch, dumps a VCD waveform.
// Waveform path: +wave=<file>  (default: pe_tb.vcd)
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module pe_tb;

    localparam int ACT_WIDTH    = 8;
    localparam int WEIGHT_WIDTH = 8;
    localparam int ACC_WIDTH    = 32;
    localparam int CLK_PERIOD   = 10; // ns

    // ---- DUT signals --------------------------------------------------------
    logic                           clk;
    logic                           rst;
    logic                           load;
    logic signed [WEIGHT_WIDTH-1:0] w;
    logic signed [ACT_WIDTH-1:0]    act;
    logic signed [ACC_WIDTH-1:0]    psum_in;
    logic        [ACC_WIDTH-1:0]    psum_out;
    logic                           valid;

    // ---- DUT ----------------------------------------------------------------
    PE #(
        .ACT_WIDTH    (ACT_WIDTH),
        .WEIGHT_WIDTH (WEIGHT_WIDTH),
        .ACC_WIDTH    (ACC_WIDTH)
    ) dut (
        .clk      (clk),
        .rst      (rst),
        .load     (load),
        .w        (w),
        .act      (act),
        .psum_in  (psum_in),
        .psum_out (psum_out),
        .valid    (valid)
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
            $display("  [FAIL] %-30s got=%0d (0x%08h)  exp=%0d (0x%08h)",
                     name, $signed(got), got, $signed(exp), exp);
        end else begin
            $display("  [PASS] %-30s = %0d (0x%08h)", name, $signed(got), got);
        end
    endtask

    task automatic checkbit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-30s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-30s = %b", name, got);
        end
    endtask

    // Drive inputs on the falling edge so they are stable, then wait for the
    // rising edge where the PE samples them and settle NBA updates (#1).
    task automatic tick(input logic r, input logic l,
                        input logic signed [WEIGHT_WIDTH-1:0] wv,
                        input logic signed [ACT_WIDTH-1:0]    av,
                        input logic signed [ACC_WIDTH-1:0]    pv);
        @(negedge clk);
        rst = r; load = l; w = wv; act = av; psum_in = pv;
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
        rst = 1'b0; load = 1'b0; w = '0; act = '0; psum_in = '0;
        tick(1'b0, 1'b0, '0, '0, '0);

        // ---- 1. Reset -------------------------------------------------------
        $display("[1] Reset");
        tick(1'b1, 1'b0, '0, '0, '0);
        check32 ("reset  psum_out", psum_out, '0);
        checkbit("reset  valid",    valid,    1'b0);

        // ---- 2. Load weight, then accumulate --------------------------------
        $display("[2] Load w=3, accumulate act=4,5");
        tick(1'b0, 1'b1, 8'sd3, 8'sd0, 32'sd0);          // load
        checkbit("load1  valid", valid, 1'b0);
        tick(1'b0, 1'b0, 8'sd0, 8'sd4, 32'sd0);          // acc: 0 + 3*4
        check32 ("acc1   psum_out", psum_out, 32'sd12);
        checkbit("acc1   valid",    valid,    1'b1);
        tick(1'b0, 1'b0, 8'sd0, 8'sd5, 32'sd12);         // acc: 12 + 3*5
        check32 ("acc2   psum_out", psum_out, 32'sd27);

        // ---- 3. Negative weight --------------------------------------------
        $display("[3] Load w=-2, accumulate act=10, -3");
        tick(1'b0, 1'b1, -8'sd2, 8'sd0, 32'sd0);
        checkbit("load2  valid", valid, 1'b0);
        tick(1'b0, 1'b0, 8'sd0, 8'sd10, 32'sd27);        // 27 + (-2*10)
        check32 ("acc3   psum_out", psum_out, 32'sd7);
        tick(1'b0, 1'b0, 8'sd0, -8'sd3, 32'sd7);         // 7 + (-2*-3)
        check32 ("acc4   psum_out", psum_out, 32'sd13);

        // ---- 4. Extreme products -------------------------------------------
        $display("[4] Extreme products");
        tick(1'b0, 1'b1, 8'sd127, 8'sd0, 32'sd0);
        tick(1'b0, 1'b0, 8'sd0, 8'sd127, 32'sd0);        // 127*127
        check32 ("max_pos psum_out", psum_out, 32'sd16129);
        tick(1'b0, 1'b1, -8'sd128, 8'sd0, 32'sd0);
        tick(1'b0, 1'b0, 8'sd0, -8'sd128, 32'sd0);       // (-128)*(-128)
        check32 ("max_neg psum_out", psum_out, 32'sd16384);

        // ---- 5. Large psum_in (no truncation) ------------------------------
        $display("[5] Large psum_in chain");
        tick(1'b0, 1'b1, 8'sd1, 8'sd0, 32'sd0);
        tick(1'b0, 1'b0, 8'sd0, 8'sd1, 32'sd1_000_000);
        check32 ("chain  psum_out", psum_out, 32'sd1_000_001);

        // ---- 6. load holds psum_out, clears valid --------------------------
        $display("[6] load holds psum_out");
        tick(1'b0, 1'b1, 8'sd9, 8'sd0, 32'sd0);          // load while psum=1000001
        check32 ("load3  psum_out holds", psum_out, 32'sd1_000_001);
        checkbit("load3  valid",          valid,    1'b0);

        // ---- 7. Asynchronous reset while busy ------------------------------
        $display("[7] Reset while busy");
        tick(1'b1, 1'b0, 8'sd0, 8'sd5, 32'sd999);
        check32 ("reset2 psum_out", psum_out, 32'sd0);
        checkbit("reset2 valid",    valid,    1'b0);
        rst = 1'b0;

        // ---- 8. Randomized cross-check vs reference model ------------------
        $display("[8] Randomized cross-check (200 cycles)");
        begin
            logic signed [WEIGHT_WIDTH-1:0] ref_w;
            logic signed [ACC_WIDTH-1:0]    ref_psum;
            logic                           ref_valid;
            logic signed [WEIGHT_WIDTH-1:0] wv;
            logic signed [ACT_WIDTH-1:0]    av;
            logic signed [ACC_WIDTH-1:0]    pv;
            logic                           lv;

            // Start from a clean reset
            tick(1'b1, 1'b0, '0, '0, '0);
            ref_w = '0; ref_psum = '0; ref_valid = 1'b0;
            rst = 1'b0;

            for (i = 0; i < 200; i++) begin
                lv = ($urandom_range(0, 9) < 2); // ~20% loads
                wv = $urandom;
                av = $urandom;
                pv = $urandom;

                // Reference model update (matches RTL priority: rst > load > acc)
                if (lv) begin
                    ref_w     = wv;
                    ref_valid = 1'b0;             // ref_psum holds
                end else begin
                    ref_psum  = pv + ACC_WIDTH'(ref_w * av);
                    ref_valid = 1'b1;
                end

                tick(1'b0, lv, wv, av, pv);
                check32 ($sformatf("rand[%0d] psum_out", i), psum_out, ref_psum);
                checkbit($sformatf("rand[%0d] valid",    i), valid,    ref_valid);
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
