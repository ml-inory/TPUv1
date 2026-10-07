//=============================================================================
// MXU (systolic array) stimulus driver + trace dumper
// DUT: rtl/mxu/mxu.sv
//
// This tb does NOT contain a golden model. It drives the DUT, checks a few
// protocol-level facts, and dumps a per-cycle trace (inputs + observed
// outputs). The reference model lives in tb/mxu/mxu_tb.py, which reads the
// trace, recomputes the expected outputs and reports PASS/FAIL.
//
//   make mxu_tb        -> runs vvp, then the Python reference check
//   +trace=<file>      trace path            (default: mxu_trace.txt)
//   +wave=<file>       VCD path              (default: mxu_tb.vcd)
//
// Trace format (one line per clock; '#' header carries the parameters):
//   cyc cmp_en rst load_weight act_in_valid psum_in_valid
//   act_in[0..ROW-1]
//   psum_in[0..COL-1]
//   weight_in[0][0..COL-1] ... weight_in[ROW-1][...]
//   psum_out[0..COL-1]                  (observed, read from dut.psum_out)
//   psum_out_valid                      (observed)
//
// Every printed check label ends with the exact signal it compares.
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module mxu_tb;

    localparam int ROW          = 3;   // deliberately non-square: catches ROW/COL mix-ups
    localparam int COL          = 4;
    localparam int ACT_WIDTH    = 8;
    localparam int WEIGHT_WIDTH = 8;
    localparam int ACC_WIDTH    = 32;
    localparam int CLK_PERIOD   = 10;  // ns
    localparam int PRIME_CYCLES = 30;  // warm-up: no comparison in Python either
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

    // ---- Bookkeeping --------------------------------------------------------
    int  errors = 0;
    int  checks = 0;
    bit  cmp_en = 1'b0;   // told to the Python model: compare from here on
    int  cyc    = 0;
    integer fd;

    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-38s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-38s = %b", name, got);
        end
    endtask

    // ---- Trace dump ---------------------------------------------------------
    // psum_out is read as dut.psum_out[j]: Icarus Verilog does not propagate an
    // unpacked-array output port up to the parent, so a tb-level copy would
    // read X even when the module itself is correct.
    task automatic dump();
        $fwrite(fd, "%0d %0d %0d %0d %0d %0d",
                cyc, cmp_en, rst, load_weight, act_in_valid, psum_in_valid);
        for (int i = 0; i < ROW; i++) $fwrite(fd, " %0d", $signed(act_in[i]));
        for (int j = 0; j < COL; j++) $fwrite(fd, " %0d", $signed(psum_in[j]));
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++) $fwrite(fd, " %0d", $signed(weight_in[i][j]));
        for (int j = 0; j < COL; j++) $fwrite(fd, " %0d", $signed(dut.psum_out[j]));
        $fwrite(fd, " %0d\n", dut.psum_out_valid);
    endtask

    // drive rst on the falling edge, advance one cycle, then dump
    task automatic step(input logic r);
        @(negedge clk);
        rst = r;
        @(posedge clk); #1;
        cyc = cyc + 1;
        dump();
    endtask

    // ---- Waveform dump / watchdog -------------------------------------------
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

    // ---- Stimulus -----------------------------------------------------------
    initial begin
        string trace_file;
        if (!$value$plusargs("trace=%s", trace_file))
            trace_file = "mxu_trace.txt";
        fd = $fopen(trace_file, "w");
        if (fd == 0)
            $fatal(1, "cannot open trace file '%s'", trace_file);
        $fwrite(fd, "# MXU ROW=%0d COL=%0d ACT=%0d WEIGHT=%0d ACC=%0d\n",
                ROW, COL, ACT_WIDTH, WEIGHT_WIDTH, ACC_WIDTH);

        // input defaults
        rst = 1'b0; load_weight = 1'b0;
        act_in_valid = 1'b0; psum_in_valid = 1'b0;
        for (int i = 0; i < ROW; i++) act_in[i]  = '0;
        for (int j = 0; j < COL; j++) psum_in[j] = '0;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++) weight_in[i][j] = '0;

        $display("=================================================");
        $display(" MXU testbench  (ROW=%0d, COL=%0d, ACT=%0d, W=%0d, ACC=%0d)",
                 ROW, COL, ACT_WIDTH, WEIGHT_WIDTH, ACC_WIDTH);
        $display(" trace -> %s   (reference model: mxu_tb.py)", trace_file);
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
        $display("[3] Warm-up (%0d cycles, act_in_valid=1)", PRIME_CYCLES);
        for (int c = 0; c < PRIME_CYCLES; c++) begin
            act_in_valid = 1'b1;
            for (int i = 0; i < ROW; i++) act_in[i]  = c*ROW + i;
            for (int j = 0; j < COL; j++) psum_in[j] = c*100 + j;
            step(1'b0);
        end
        check_bit("warm-up psum_out_valid", psum_out_valid, 1'b1);
        cmp_en = 1'b1;

        // ---- [4] Randomized stream (Python reference compares this window) --
        $display("[4] Randomized stream (%0d cycles) vs Python reference model",
                 RUN_CYCLES);
        for (int c = 0; c < RUN_CYCLES; c++) begin
            if (c % 61 == 0) begin
                for (int i = 0; i < ROW; i++)
                    for (int j = 0; j < COL; j++) weight_in[i][j] = $urandom;
                act_in_valid = 1'b0;
                load_weight  = 1'b1;
                step(1'b0);
                load_weight  = 1'b0;
            end else if (c % 97 == 0) begin
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

        $fclose(fd);

        // ---- Directed-check summary (data checking is done by mxu_tb.py) ----
        $display("=================================================");
        if (errors == 0)
            $display(" protocol checks PASSED (%0d checks) - see mxu_tb.py for data check", checks);
        else
            $display(" protocol checks FAILED (%0d/%0d)", errors, checks);
        $display("=================================================");
        if (errors != 0) $fatal(1, "MXU tb protocol checks failed");
        $finish;
    end

endmodule

`default_nettype wire
