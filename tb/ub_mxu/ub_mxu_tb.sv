//=============================================================================
// UB -> MXU integration testbench
//
// DUTs: rtl/ub.sv (act storage) + rtl/mxu/{mxu,pe,delay_chain}.sv
//
// What is exercised
//   * the act matrix A (K x ROW) is written into UB through its write port
//     and read back through the read port (read-back check),
//   * UB is line-addressed (WIDTH*WORD_SIZE bits per read), so one line holds
//     a whole activation vector: line k = A[k][0..ROW-1]. The tb plays the
//     "act fetch + dispatch" role: it reads one line per activation vector
//     and broadcasts the whole line to the ROW act inputs in the next cycle,
//     so the source delivers one activation vector per cycle. The row skew
//     comes from the array itself now (rtl/mxu/delay_chain.sv is a DEPTH-deep
//     shift register, so row i sees the vector i cycles later).
//   * act_in_valid is held high through the whole stream, so each PE updates
//     every cycle and the array output is the sliding convolution
//         psum_out[j][c] = sum_i W[i][j] * act_hist[i][c+1-ROW-j]
//     which the tb checks cycle by cycle, and
//   * at cycle r0 + k + ROW + j (r0 = cycle that issued the read of line 0)
//     the array must present the end-to-end result dot(A[k][:], W[:,j]) on
//     psum_out[j] with psum_out_valid = 1.
//
// The simulation runs two rounds (fresh reset, new act image in UB, new
// weights) with signed act/weight values, so the negative-product path and
// weight reload are covered too.
//
//   make ub_mxu_tb   -> runs the tb (self-checking; no Python model needed)
//   +wave=<file>     VCD path (default ub_mxu_tb.vcd)
//
// Icarus note: psum_out is read as u_mxu.psum_out[j]; an unpacked-array output
// port does not propagate up to a tb-level copy (same caveat as mxu_tb.sv).
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module ub_mxu_tb;

    localparam int ROW          = 4;      // act rows of the array
    localparam int COL          = 3;      // deliberately non-square
    localparam int K            = 4;      // activation vectors (=matmul depth)
    localparam int ACT_WIDTH    = 8;
    localparam int WEIGHT_WIDTH = 8;
    localparam int ACC_WIDTH    = 32;
    localparam int LINE         = ACT_WIDTH*ROW;   // UB line = one act vector
    localparam int UB_DEPTH     = 64;     // acts live in lines 0..K-1
    localparam int UB_AW        = $clog2(UB_DEPTH);
    localparam int PRIME        = 16;     // zero-act cycles to arm + clear the array
    localparam int CLK_PERIOD   = 10;     // ns
    localparam int MAXC         = 600;    // history depth (cycles)

    // ---- clock / reset ------------------------------------------------------
    logic clk, rst;
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- UB signals ---------------------------------------------------------
    logic               ub_rd_en;
    logic [UB_AW-1:0]   ub_rd_addr;
    logic [LINE-1:0]    ub_rd_data;
    logic               ub_wr_en;
    logic [UB_AW-1:0]   ub_wr_addr;
    logic [LINE-1:0]    ub_wr_data;

    // ---- MXU signals --------------------------------------------------------
    logic                     load_weight;
    logic [WEIGHT_WIDTH-1:0]  weight_in [0:ROW-1][0:COL-1];
    logic                     act_in_valid;
    logic [ACT_WIDTH-1:0]     act_in    [0:ROW-1];
    logic                     psum_in_valid;
    logic [ACC_WIDTH-1:0]     psum_in   [0:COL-1];
    logic                     psum_out_valid;
    logic [ACC_WIDTH-1:0]     psum_out  [0:COL-1];

    // ---- DUTs ---------------------------------------------------------------
    UB #(
        .WIDTH(ACT_WIDTH), .WORD_SIZE(ROW), .DEPTH(UB_DEPTH)
    ) u_ub (
        .clk     (clk),
        .rst     (rst),
        .rd_en   (ub_rd_en),
        .rd_addr (ub_rd_addr),
        .rd_data (ub_rd_data),
        .wr_en   (ub_wr_en),
        .wr_addr (ub_wr_addr),
        .wr_data (ub_wr_data)
    );

    MXU #(
        .ROW(ROW), .COL(COL),
        .ACT_WIDTH(ACT_WIDTH), .WEIGHT_WIDTH(WEIGHT_WIDTH), .ACC_WIDTH(ACC_WIDTH)
    ) u_mxu (
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

    // ---- Golden data / bookkeeping -----------------------------------------
    int  A [0:K-1][0:ROW-1];              // act matrix held in UB (signed)
    int  W [0:ROW-1][0:COL-1];            // weights (signed)
    int  cyc = 0;
    int  checks = 0;
    int  errors = 0;
    int  errs0;
    int  r0;                              // cycle that issued the UB read of A[0][0]
    int  sb_start, sb_end;                // waveform-model comparison window

    logic [ACT_WIDTH-1:0]  act_hist   [0:ROW-1][0:MAXC];
    logic [ACC_WIDTH-1:0]  psum_hist  [0:COL-1][0:MAXC];
    bit                    pvalid_hist [0:MAXC];

    // ---- UB line helpers ----------------------------------------------------
    // line k of UB holds A[k][0..ROW-1], word i at bits i*ACT_WIDTH +: ACT_WIDTH
    function automatic logic [LINE-1:0] act_line(input int k);
        logic [LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++)
            l[i*ACT_WIDTH +: ACT_WIDTH] = ACT_WIDTH'(A[k][i]);
        act_line = l;
    endfunction

    // every lane = 0xA5: an unused line, so a wrong address shows up at once
    function automatic logic [LINE-1:0] sentinel_line();
        logic [LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++) l[i*ACT_WIDTH +: ACT_WIDTH] = 8'hA5;
        sentinel_line = l;
    endfunction

    // index of the first word that differs (-1 when the lines match)
    function automatic int bad_lane(input logic [LINE-1:0] got,
                                    input logic [LINE-1:0] exp);
        bad_lane = -1;
        for (int i = 0; i < ROW; i++)
            if (bad_lane < 0 && got[i*ACT_WIDTH +: ACT_WIDTH] !== exp[i*ACT_WIDTH +: ACT_WIDTH])
                bad_lane = i;
    endfunction

    // ---- Waveform dump / watchdog -------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "ub_mxu_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, ub_mxu_tb);
    end

    initial begin
        #(CLK_PERIOD * 100000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- Helpers ------------------------------------------------------------
    task automatic check_line(input string name, input logic [LINE-1:0] got,
                              input logic [LINE-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-44s got=%h exp=%h (first bad word %0d)",
                     name, got, exp, bad_lane(got, exp));
        end
    endtask

    // drive inputs at the falling edge, sample after the rising edge
    task automatic edge_in();
        @(negedge clk);
    endtask

    task automatic tick();
        @(posedge clk); #1;
        cyc++;
        for (int i = 0; i < ROW; i++) act_hist[i][cyc] = act_in[i];
        for (int j = 0; j < COL; j++) psum_hist[j][cyc] = u_mxu.psum_out[j];
        pvalid_hist[cyc] = u_mxu.psum_out_valid;
    endtask

    // ---- Test phases --------------------------------------------------------
    task automatic reset_both(input int ncycles);
        edge_in(); rst = 1'b1;
        for (int c = 0; c < ncycles; c++) tick();
        edge_in(); rst = 1'b0; tick();
    endtask

    // write the K act lines, then 0xA5 sentinel lines over the rest of UB
    task automatic ub_write_image();
        for (int a = 0; a < UB_DEPTH; a++) begin
            edge_in();
            ub_wr_en   = 1'b1;
            ub_wr_addr = UB_AW'(a);
            ub_wr_data = (a < K) ? act_line(a) : sentinel_line();
            tick();
        end
        edge_in(); ub_wr_en = 1'b0; tick();
    endtask

    // read the act lines back out of UB (they are what the array will see)
    task automatic ub_readback_check();
        errs0 = errors;
        for (int a = 0; a < K; a++) begin
            edge_in(); ub_rd_en = 1'b1; ub_rd_addr = UB_AW'(a); tick();
            check_line($sformatf("UB line[%0d] read-back", a), ub_rd_data,
                       act_line(a));
        end
        // a couple of sentinel lines: catches address aliasing at the end
        for (int a = K; a < UB_DEPTH; a += UB_DEPTH-1-K) begin
            edge_in(); ub_rd_en = 1'b1; ub_rd_addr = UB_AW'(a); tick();
            check_line($sformatf("UB line[%0d] sentinel", a), ub_rd_data,
                       sentinel_line());
        end
        edge_in(); ub_rd_en = 1'b0; ub_rd_addr = '0; tick();
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-44s (%0d lines + 2 sentinels)",
                     "UB holds the act image", K);
    endtask

    task automatic load_weights();
        edge_in();
        load_weight = 1'b1;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++) weight_in[i][j] = WEIGHT_WIDTH'(W[i][j]);
        tick();
        edge_in(); load_weight = 1'b0; tick();
    endtask

    // zero acts with act_in_valid high: arms every delay chain and clears the
    // PE psum chain (each PE computes 0 + w*0 = 0), so the array starts clean
    task automatic prime_array();
        edge_in();
        act_in_valid = 1'b1;
        for (int i = 0; i < ROW; i++) act_in[i] = '0;
        tick();
        for (int c = 0; c < PRIME; c++) begin edge_in(); tick(); end
        sb_start = cyc + 1;
    endtask

    // One UB line read per activation vector, the whole line is broadcast to
    // the ROW act inputs on the next cycle:
    //   line k is read in cycle r0 + k  (1-cycle UB read latency),
    //   all ROW words are driven in cycle r0 + 1 + k,
    // so the source delivers one activation vector per cycle and the MXU skews
    // the rows internally (row i sees the vector i cycles later).
    task automatic stream_acts();
        for (int k = 0; k < K; k++) begin
            edge_in();
            // broadcast the line fetched last cycle (1-cycle UB read latency)
            if (k > 0)
                for (int i = 0; i < ROW; i++)
                    act_in[i] = ub_rd_data[i*ACT_WIDTH +: ACT_WIDTH];
            ub_rd_en   = 1'b1;
            ub_rd_addr = UB_AW'(k);
            tick();
            if (k == 0) r0 = cyc;               // cycle that issued the read
        end
        // last vector: broadcast it and stop reading
        edge_in();
        for (int i = 0; i < ROW; i++)
            act_in[i] = ub_rd_data[i*ACT_WIDTH +: ACT_WIDTH];
        ub_rd_en   = 1'b0;
        ub_rd_addr = '0;
        tick();
        // flush with zeros while the last wavefront walks to all columns
        for (int c = 0; c < ROW + COL + 2; c++) begin
            edge_in();
            for (int i = 0; i < ROW; i++) act_in[i] = '0;
            tick();
        end
        sb_end = cyc;
        edge_in(); act_in_valid = 1'b0; tick();
    endtask

    // (a) every cycle of the stream against the waveform model,
    // (b) every wavefront/column against dot(A[k][:], W[:,j])
    task automatic check_round(input int round);
        int cc, acc, sb_err, gerr;
        int gemm_gold [0:K-1][0:COL-1];

        sb_err = 0;
        checks += (sb_end - sb_start + 1) * COL;
        for (int c = sb_start; c <= sb_end; c++) begin
            for (int j = 0; j < COL; j++) begin
                acc = 0;
                for (int i = 0; i < ROW; i++)
                    acc += W[i][j] * $signed(act_hist[i][c + 1 - ROW - j]);
                if ($signed(psum_hist[j][c]) !== acc) begin
                    sb_err++;
                    if (sb_err <= 6)
                        $display("  [FAIL] round %0d cyc %0d col %0d: psum_out=%0d, UB-fed model=%0d",
                                 round, c, j, $signed(psum_hist[j][c]), acc);
                end
            end
        end
        errors += sb_err;
        if (sb_err == 0)
            $display("  [PASS] %-44s (%0d cycles x %0d cols)",
                     "MXU follows the UB-fed waveform model",
                     sb_end - sb_start + 1, COL);

        gerr = 0;
        for (int k = 0; k < K; k++) begin
            for (int j = 0; j < COL; j++) begin
                acc = 0;
                for (int i = 0; i < ROW; i++) acc += W[i][j] * A[k][i];
                gemm_gold[k][j] = acc;
                cc = r0 + k + ROW + j;         // dot product lands here
                checks++;
                if ($signed(psum_hist[j][cc]) !== acc || !pvalid_hist[cc]) begin
                    errors++; gerr++;
                    $display("  [FAIL] round %0d wavefront %0d col %0d: cyc %0d psum_out=%0d valid=%b, exp=%0d",
                             round, k, j, cc, $signed(psum_hist[j][cc]),
                             pvalid_hist[cc], acc);
                end else begin
                    $display("  [PASS] round %0d wavefront %0d col %0d = %0d (cyc %0d, valid=1)",
                             round, k, j, acc, cc);
                end
            end
        end

        $display("       round %0d GEMM result  A(%0d x %0d) * W(%0d x %0d):",
                 round, K, ROW, ROW, COL);
        for (int k = 0; k < K; k++) begin
            $write("         wavefront %0d  observed:", k);
            for (int j = 0; j < COL; j++)
                $write(" %5d", $signed(psum_hist[j][r0 + k + ROW + j]));
            $write("   golden:");
            for (int j = 0; j < COL; j++) $write(" %5d", gemm_gold[k][j]);
            $write("\n");
        end
        if (gerr != 0) $display("       (%0d wavefront/column mismatches)", gerr);
    endtask

    // ---- Stimulus -----------------------------------------------------------
    initial begin
        // default inputs
        rst = 1'b0;
        ub_rd_en = 1'b0; ub_rd_addr = '0; ub_wr_en = 1'b0;
        ub_wr_addr = '0; ub_wr_data = '0;
        load_weight = 1'b0; act_in_valid = 1'b0; psum_in_valid = 1'b0;
        for (int i = 0; i < ROW; i++) begin
            act_in[i] = '0;
            for (int j = 0; j < COL; j++) weight_in[i][j] = '0;
        end
        for (int j = 0; j < COL; j++) psum_in[j] = '0;

        $display("=============================================================");
        $display(" UB -> MXU integration tb");
        $display("   UB : rtl/ub.sv (WIDTH=%0d, DEPTH=%0d)", ACT_WIDTH, UB_DEPTH);
        $display("   MXU: rtl/mxu (ROW=%0d, COL=%0d, K=%0d, signed %0d-bit acts/weights)",
                 ROW, COL, K, ACT_WIDTH);
        $display("=============================================================");

        for (int round = 0; round < 2; round++) begin
            // signed, pattern-dependent data: round 1 differs from round 0
            for (int k = 0; k < K; k++)
                for (int i = 0; i < ROW; i++)
                    A[k][i] = (round == 0) ? (((k*ROW + i) * 5) % 9) - 4
                                           : (((k*ROW + i) * 7) % 11) - 5;
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    W[i][j] = (round == 0) ? (((i*COL + j) * 3) % 7) - 3
                                           : (((i*COL + j) * 5) % 9) - 4;

            $display("[round %0d] reset, load act image into UB, load weights, stream",
                     round);
            reset_both(2);
            ub_write_image();
            ub_readback_check();
            load_weights();
            prime_array();
            stream_acts();
            check_round(round);
        end

        $display("=============================================================");
        $display(" UB -> MXU tb summary: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display(" *** UB -> MXU TB PASSED ***");
        else
            $display(" *** UB -> MXU TB FAILED ***");
        $display("=============================================================");
        if (errors != 0) $fatal(1, "UB -> MXU tb failed");
        $finish;
    end

endmodule

`default_nettype wire
