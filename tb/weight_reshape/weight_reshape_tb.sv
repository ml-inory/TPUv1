//=============================================================================
// WeightReshape testbench
// DUT: rtl/weight_reshape.sv - combinational "reshape" of one packed weight
//      line from the WeightFIFO into the MXU weight-port layout.
//
// Layout pinned by this tb (row-major, the convention the WeightFIFO /
// MXU integration uses):
//     din[(i*COL + j)*WIDTH +: WIDTH]  ->  dout[i][j]
// i.e. word q of the packed input lands on dout[q/COL][q%COL].
// A transposed (column-major) implementation fails on the non-square
// instances below.
//
// Coverage: three parameter sets, all lanes compared on every vector
//   * u1: WIDTH=8,  ROW=4, COL=3  (the MXU/WeightFIFO configuration)
//   * u2: WIDTH=16, ROW=3, COL=2  (different word width, non-square)
//   * u3: WIDTH=8,  ROW=1, COL=1  (single word corner)
// Patterns: all-0, all-1, MSB-only, LSB-only, 0xAA.., 0x55.., distinct per
// word, 0x7F/0x80 alternating, plus 200 random lines per instance.
//
//   make weight_reshape_tb   -> runs the tb (self-checking)
//   +wave=<file>             VCD path (default weight_reshape_tb.vcd)
//
// Icarus note: dout is a two-dimensional unpacked-array output port; the
// iverilog elaborator asserts when such a port is connected to a tb-level
// array, and a tb-level copy of an unpacked-array output reads X. The port is
// therefore left unconnected here and every lane is read through the DUT
// hierarchy as u1.dout[i][j] (same trick as mxu_tb.sv, which reads
// dut.psum_out[j]).
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module weight_reshape_tb;

    // ---- parameter sets -----------------------------------------------------
    localparam int W1 = 8,  R1 = 4, C1 = 3, N1 = W1*R1*C1;   // 96-bit line
    localparam int W2 = 16, R2 = 3, C2 = 2, N2 = W2*R2*C2;   // 96-bit line
    localparam int W3 = 8,  R3 = 1, C3 = 1, N3 = W3*R3*C3;   // 8-bit line

    localparam int N_RAND = 200;

    // ---- DUT signals --------------------------------------------------------
    logic [N1-1:0] din1;
    logic [N2-1:0] din2;
    logic [N3-1:0] din3;

    // ---- DUTs ---------------------------------------------------------------
    WeightReshape #(.WIDTH(W1), .ROW(R1), .COL(C1)) u1 (.din(din1), .dout());
    WeightReshape #(.WIDTH(W2), .ROW(R2), .COL(C2)) u2 (.din(din2), .dout());
    WeightReshape #(.WIDTH(W3), .ROW(R3), .COL(C3)) u3 (.din(din3), .dout());

    // ---- Bookkeeping --------------------------------------------------------
    int errors = 0;
    int checks = 0;

    // expected weight arrays (the golden image per instance)
    int Wm1 [0:R1-1][0:C1-1];
    int Wm2 [0:R2-1][0:C2-1];
    int Wm3 [0:R3-1][0:C3-1];

    // ---- pattern generator --------------------------------------------------
    function automatic string mode_name(input int mode);
        case (mode)
            0: mode_name = "all 0";
            1: mode_name = "all 1";
            2: mode_name = "MSB only";
            3: mode_name = "LSB only";
            4: mode_name = "0xAA..";
            5: mode_name = "0x55..";
            6: mode_name = "distinct";
            7: mode_name = "random";
            default: mode_name = "0x7F/0x80";
        endcase
    endfunction

    function automatic logic [31:0] pat_word(input int mode, input int q,
                                             input int width, input int salt);
        logic [31:0] v;
        v = '0;
        case (mode)
            0: v = '0;
            1: v = (32'd1 << width) - 32'd1;
            2: v = 32'd1 << (width-1);
            3: v = 32'd1;
            4: for (int b = 1; b < width; b += 2) v[b] = 1'b1;
            5: for (int b = 0; b < width; b += 2) v[b] = 1'b1;
            6: v = q*37 + 11 + salt;
            7: v = $urandom;
            default: v = ((q % 2) == 0) ? ((32'd1 << (width-1)) - 32'd1)
                                        : (32'd1 << (width-1));
        endcase
        pat_word = v;
    endfunction

    // ---- instance 1: WIDTH=8, ROW=4, COL=3 ----------------------------------
    task automatic fill1(input int mode, input int salt);
        for (int i = 0; i < R1; i++)
            for (int j = 0; j < C1; j++)
                Wm1[i][j] = int'(pat_word(mode, i*C1 + j, W1, salt));
    endtask

    function automatic logic [N1-1:0] pack1();
        pack1 = '0;
        for (int i = 0; i < R1; i++)
            for (int j = 0; j < C1; j++)
                pack1[(i*C1 + j)*W1 +: W1] = W1'(Wm1[i][j]);
    endfunction

    task automatic check1(input string name, input bit quiet);
        bit bad;
        bad = 1'b0;
        checks++;
        for (int i = 0; i < R1; i++)
            for (int j = 0; j < C1; j++)
                if (u1.dout[i][j] !== W1'(Wm1[i][j])) begin
                    bad = 1'b1;
                    errors++;
                    if (errors <= 30)
                        $display("  [FAIL] %-30s dout[%0d][%0d] got=%02h exp=%02h",
                                 name, i, j, u1.dout[i][j], W1'(Wm1[i][j]));
                end
        if (!bad && !quiet)
            $display("  [PASS] %-30s (%0d lanes)", name, R1*C1);
    endtask

    // ---- instance 2: WIDTH=16, ROW=3, COL=2 ---------------------------------
    task automatic fill2(input int mode, input int salt);
        for (int i = 0; i < R2; i++)
            for (int j = 0; j < C2; j++)
                Wm2[i][j] = int'(pat_word(mode, i*C2 + j, W2, salt));
    endtask

    function automatic logic [N2-1:0] pack2();
        pack2 = '0;
        for (int i = 0; i < R2; i++)
            for (int j = 0; j < C2; j++)
                pack2[(i*C2 + j)*W2 +: W2] = W2'(Wm2[i][j]);
    endfunction

    task automatic check2(input string name, input bit quiet);
        bit bad;
        bad = 1'b0;
        checks++;
        for (int i = 0; i < R2; i++)
            for (int j = 0; j < C2; j++)
                if (u2.dout[i][j] !== W2'(Wm2[i][j])) begin
                    bad = 1'b1;
                    errors++;
                    if (errors <= 30)
                        $display("  [FAIL] %-30s dout[%0d][%0d] got=%04h exp=%04h",
                                 name, i, j, u2.dout[i][j], W2'(Wm2[i][j]));
                end
        if (!bad && !quiet)
            $display("  [PASS] %-30s (%0d lanes)", name, R2*C2);
    endtask

    // ---- instance 3: WIDTH=8, ROW=1, COL=1 ----------------------------------
    task automatic fill3(input int mode, input int salt);
        for (int i = 0; i < R3; i++)
            for (int j = 0; j < C3; j++)
                Wm3[i][j] = int'(pat_word(mode, i*C3 + j, W3, salt));
    endtask

    function automatic logic [N3-1:0] pack3();
        pack3 = '0;
        for (int i = 0; i < R3; i++)
            for (int j = 0; j < C3; j++)
                pack3[(i*C3 + j)*W3 +: W3] = W3'(Wm3[i][j]);
    endfunction

    task automatic check3(input string name, input bit quiet);
        bit bad;
        bad = 1'b0;
        checks++;
        for (int i = 0; i < R3; i++)
            for (int j = 0; j < C3; j++)
                if (u3.dout[i][j] !== W3'(Wm3[i][j])) begin
                    bad = 1'b1;
                    errors++;
                    if (errors <= 30)
                        $display("  [FAIL] %-30s dout[%0d][%0d] got=%02h exp=%02h",
                                 name, i, j, u3.dout[i][j], W3'(Wm3[i][j]));
                end
        if (!bad && !quiet)
            $display("  [PASS] %-30s (%0d lanes)", name, R3*C3);
    endtask

    // ---- Waveform dump ------------------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "weight_reshape_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, weight_reshape_tb);
    end

    // ---- Stimulus -----------------------------------------------------------
    initial begin
        din1 = '0; din2 = '0; din3 = '0;

        $display("=============================================================");
        $display(" WeightReshape testbench (din[(i*COL+j)*WIDTH +: WIDTH] -> dout[i][j])");
        $display("   u1: WIDTH=%0d ROW=%0d COL=%0d (%0d-bit line)", W1, R1, C1, N1);
        $display("   u2: WIDTH=%0d ROW=%0d COL=%0d (%0d-bit line)", W2, R2, C2, N2);
        $display("   u3: WIDTH=%0d ROW=%0d COL=%0d (%0d-bit line)", W3, R3, C3, N3);
        $display("=============================================================");

        // ---- directed patterns ----------------------------------------------
        $display("[1] Directed patterns (all lanes compared every vector)");
        for (int mode = 0; mode <= 8; mode++) begin
            fill1(mode, mode*3); din1 = pack1(); #1;
            check1($sformatf("u1 %s", mode_name(mode)), 1'b0);

            fill2(mode, mode*5); din2 = pack2(); #1;
            check2($sformatf("u2 %s", mode_name(mode)), 1'b0);

            fill3(mode, mode*7); din3 = pack3(); #1;
            check3($sformatf("u3 %s", mode_name(mode)), 1'b0);
        end

        // ---- randomized lines -----------------------------------------------
        $display("[2] %0d random lines per instance", N_RAND);
        for (int c = 0; c < N_RAND; c++) begin
            fill1(7, c); din1 = pack1(); #1;
            check1($sformatf("u1 random %0d", c), 1'b1);

            fill2(7, c); din2 = pack2(); #1;
            check2($sformatf("u2 random %0d", c), 1'b1);

            fill3(7, c); din3 = pack3(); #1;
            check3($sformatf("u3 random %0d", c), 1'b1);
        end
        checks++;
        $display("  [%s] %-30s (%0d random lines x 3 instances)",
                 (errors == 0) ? "PASS" : "INFO", "random sweep done", N_RAND);

        // ---- Summary --------------------------------------------------------
        $display("=============================================================");
        $display(" WeightReshape tb summary: %0d vector checks, %0d lane failures",
                 checks, errors);
        if (errors == 0)
            $display(" *** WEIGHT RESHAPE TB PASSED ***");
        else
            $display(" *** WEIGHT RESHAPE TB FAILED ***");
        $display("=============================================================");
        if (errors != 0) $fatal(1, "WeightReshape tb failed");
        $finish;
    end

endmodule

`default_nettype wire
