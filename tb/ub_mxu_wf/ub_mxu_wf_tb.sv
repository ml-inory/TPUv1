//=============================================================================
// UB + MXU + WeightFIFO integration testbench
//
// DUTs: rtl/ub.sv, rtl/weight_fifo.sv, rtl/weight_reshape.sv,
//       rtl/mxu/{mxu,pe,delay_chain}.sv
//
// Dataflow exercised (the weight / activation path of the TPU):
//   * one WeightFIFO entry holds a whole ROW x COL weight tile
//     (WEIGHT_WIDTH*ROW*COL bits); tiles are prefetched into the FIFO, popped
//     one per round, reshaped by WeightReshape into the MXU weight-port layout
//     and loaded with load_weight
//   * one activation matrix per round is written into UB (one UB line = one
//     activation vector = ROW words) and streamed into the MXU act ports, one
//     vector per cycle (the array skews the rows internally - delay_chain is
//     a DEPTH-deep shift register)
//   * while round 0 computes, the next weight tile is pushed into the FIFO
//     (prefetch during compute); the FIFO must hand back the tiles in order
//   * every round: UB image read back, FIFO entry compared with the golden
//     tile, every reshaped weight lane compared with the golden tile, waveform
//     model checked cycle by cycle and psum_out compared with A_round x W_tile
//     at the cycle the wavefront lands
//   * WeightFIFO full/empty transitions are checked on the way
//
//   make ub_mxu_wf_tb   -> runs the tb (self-checking; no Python model needed)
//   +wave=<file>        VCD path (default ub_mxu_wf_tb.vcd)
//
// Icarus note: psum_out is read as u_mxu.psum_out[j]; an unpacked-array output
// port does not propagate up to a tb-level copy (same caveat as mxu_tb.sv).
// For the same reason WeightReshape's dout port is left unconnected and its
// lanes are copied into the MXU weight_in array through a hierarchical
// reference (in real RTL this is just .weight_in(u_reshape.dout)).
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module ub_mxu_wf_tb;

    localparam int ROW          = 4;      // act rows of the array
    localparam int COL          = 3;      // deliberately non-square
    localparam int K            = 4;      // activation vectors per round
    localparam int NT           = 3;      // weight tiles / rounds
    localparam int ACT_WIDTH    = 8;
    localparam int WEIGHT_WIDTH = 8;
    localparam int ACC_WIDTH    = 32;

    localparam int ACT_LINE     = ACT_WIDTH*ROW;      // UB line = one act vector
    localparam int UB_DEPTH     = 64;
    localparam int UB_AW        = $clog2(UB_DEPTH);

    localparam int WT_WORDS     = ROW*COL;            // weights per tile
    localparam int WF_LINE      = WEIGHT_WIDTH*WT_WORDS;  // FIFO entry = one tile
    localparam int WF_TILES     = 2;                  // FIFO depth (tiles)

    localparam int PRIME        = 16;     // zero-act cycles to clear the array
    localparam int CLK_PERIOD   = 10;     // ns
    localparam int MAXC         = 900;    // history depth (cycles)

    // ---- clock / reset ------------------------------------------------------
    logic clk, rst;
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- UB signals ---------------------------------------------------------
    logic                 ub_rd_en;
    logic [UB_AW-1:0]     ub_rd_addr;
    logic [ACT_LINE-1:0]  ub_rd_data;
    logic                 ub_wr_en;
    logic [UB_AW-1:0]     ub_wr_addr;
    logic [ACT_LINE-1:0]  ub_wr_data;

    // ---- WeightFIFO signals -------------------------------------------------
    logic                 wf_wr_en;
    logic [WF_LINE-1:0]   wf_wr_data;
    logic                 wf_full;
    logic                 wf_rd_en;
    logic [WF_LINE-1:0]   wf_rd_data;
    logic                 wf_empty;

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

    WeightFIFO #(
        .TILE(WF_TILES), .WIDTH(WEIGHT_WIDTH), .DEPTH(WT_WORDS)
    ) u_wf (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (wf_wr_en),
        .wr_data (wf_wr_data),
        .full    (wf_full),
        .rd_en   (wf_rd_en),
        .rd_data (wf_rd_data),
        .empty   (wf_empty)
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

    // ---- weight loader: FIFO entry -> WeightReshape -> MXU weight ports ------
    WeightReshape #(
        .WIDTH(WEIGHT_WIDTH), .ROW(ROW), .COL(COL)
    ) u_reshape (
        .din  (wf_rd_data),
        .dout ()
    );

    // Icarus quirk: an output port that is an unpacked array does not drive a
    // tb-level array (it would read X), so the reshaped lanes are copied over
    // through the hierarchy. In real RTL this is a plain wire connection:
    //     .weight_in(u_reshape.dout)
    always_comb begin
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                weight_in[i][j] = u_reshape.dout[i][j];
    end

    // ---- Golden data / bookkeeping -----------------------------------------
    int  A [0:NT-1][0:K-1][0:ROW-1];          // act images (signed)
    int  W [0:NT-1][0:ROW-1][0:COL-1];        // weight tiles (signed)
    int  Wcur [0:ROW-1][0:COL-1];             // weights loaded in this round
    int  cyc = 0;
    int  checks = 0;
    int  errors = 0;
    int  errs0;
    int  r0;                                  // cycle that issued the UB read
    int  sb_start, sb_end;                    // waveform-model window

    logic [ACT_WIDTH-1:0]  act_hist   [0:ROW-1][0:MAXC];
    logic [ACC_WIDTH-1:0]  psum_hist  [0:COL-1][0:MAXC];
    bit                    pvalid_hist [0:MAXC];

    // ---- helpers ------------------------------------------------------------
    // UB line k of round r = the k-th activation vector of that round
    function automatic logic [ACT_LINE-1:0] act_line(input int r, input int k);
        logic [ACT_LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++)
            l[i*ACT_WIDTH +: ACT_WIDTH] = ACT_WIDTH'(A[r][k][i]);
        act_line = l;
    endfunction

    // unused UB lines carry 0xA5 in every word, so a wrong address shows up
    function automatic logic [ACT_LINE-1:0] sentinel_line();
        logic [ACT_LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++) l[i*ACT_WIDTH +: ACT_WIDTH] = 8'hA5;
        sentinel_line = l;
    endfunction

    // WeightFIFO entry of tile t: word (i*COL+j) = W[t][i][j]
    function automatic logic [WF_LINE-1:0] pack_w(input int t);
        logic [WF_LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                l[(i*COL + j)*WEIGHT_WIDTH +: WEIGHT_WIDTH] = WEIGHT_WIDTH'(W[t][i][j]);
        pack_w = l;
    endfunction

    function automatic int bad_lane(input logic [ACT_LINE-1:0] got,
                                    input logic [ACT_LINE-1:0] exp);
        bad_lane = -1;
        for (int i = 0; i < ROW; i++)
            if (bad_lane < 0 && got[i*ACT_WIDTH +: ACT_WIDTH] !== exp[i*ACT_WIDTH +: ACT_WIDTH])
                bad_lane = i;
    endfunction

    function automatic int bad_wword(input logic [WF_LINE-1:0] got,
                                     input logic [WF_LINE-1:0] exp);
        bad_wword = -1;
        for (int q = 0; q < WT_WORDS; q++)
            if (bad_wword < 0 && got[q*WEIGHT_WIDTH +: WEIGHT_WIDTH] !== exp[q*WEIGHT_WIDTH +: WEIGHT_WIDTH])
                bad_wword = q;
    endfunction

    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-46s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-46s = %b", name, got);
        end
    endtask

    task automatic check_acts(input string name, input logic [ACT_LINE-1:0] got,
                              input logic [ACT_LINE-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-46s got=%h exp=%h (first bad row %0d)",
                     name, got, exp, bad_lane(got, exp));
        end else begin
            $display("  [PASS] %-46s = %h", name, got);
        end
    endtask

    task automatic check_wline(input string name, input logic [WF_LINE-1:0] got,
                               input logic [WF_LINE-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-46s got=%h exp=%h (first bad word %0d)",
                     name, got, exp, bad_wword(got, exp));
        end else begin
            $display("  [PASS] %-46s = %h", name, got);
        end
    endtask

    // ---- clock helpers ------------------------------------------------------
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
    task automatic reset_all();
        edge_in(); rst = 1'b1;
        tick(); tick();
        edge_in(); rst = 1'b0;
        tick();
    endtask

    // act image of round r into UB, plus 0xA5 sentinel lines at K and DEPTH-1
    task automatic ub_write_image(input int r);
        for (int a = 0; a < UB_DEPTH; a++) begin
            edge_in();
            ub_wr_en   = 1'b1;
            ub_wr_addr = UB_AW'(a);
            if (a < K)                       ub_wr_data = act_line(r, a);
            else if (a == K || a == UB_DEPTH-1) ub_wr_data = sentinel_line();
            else                              ub_wr_data = '0;
            tick();
        end
        edge_in(); ub_wr_en = 1'b0; tick();
    endtask

    task automatic ub_readback(input int r);
        errs0 = errors;
        for (int a = 0; a < K; a++) begin
            edge_in(); ub_rd_en = 1'b1; ub_rd_addr = UB_AW'(a); tick();
            check_acts($sformatf("round %0d UB line[%0d]", r, a), ub_rd_data, act_line(r, a));
        end
        for (int a = K; a < UB_DEPTH; a += UB_DEPTH-1-K) begin
            edge_in(); ub_rd_en = 1'b1; ub_rd_addr = UB_AW'(a); tick();
            check_acts($sformatf("round %0d UB sentinel[%0d]", r, a), ub_rd_data, sentinel_line());
        end
        edge_in(); ub_rd_en = 1'b0; ub_rd_addr = '0; tick();
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-46s (%0d lines + 2 sentinels)", "UB holds the act image", K);
    endtask

    task automatic wf_push(input int t);
        edge_in();
        wf_wr_en   = 1'b1;
        wf_wr_data = pack_w(t);
        tick();
        edge_in(); wf_wr_en = 1'b0; tick();
    endtask

    // the reshaped lanes must reproduce the golden tile (WeightReshape sits
    // directly in the weight path now, so it is checked in situ)
    task automatic check_reshape(input int r);
        bit bad;
        bad = 1'b0;
        checks++;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                if (u_reshape.dout[i][j] !== WEIGHT_WIDTH'(Wcur[i][j])) begin
                    bad = 1'b1;
                    errors++;
                    if (errors <= 20)
                        $display("  [FAIL] round %0d WeightReshape lane[%0d][%0d] got=%02h exp=%02h",
                                 r, i, j, u_reshape.dout[i][j], WEIGHT_WIDTH'(Wcur[i][j]));
                end
        if (!bad)
            $display("  [PASS] round %0d %-38s (%0d lanes)", r,
                     "WeightReshape tile matches", ROW*COL);
    endtask

    // pop tile r, compare the FIFO output with the golden tile, load it
    task automatic wf_pop_and_load(input int r);
        edge_in();
        wf_rd_en = 1'b1;
        tick();                                  // rd_data = the popped entry
        check_wline($sformatf("round %0d WeightFIFO entry (tile %0d)", r, r),
                    wf_rd_data, pack_w(r));
        check_reshape(r);                        // FIFO entry -> weight lanes
        edge_in(); wf_rd_en = 1'b0; tick();
        // the entry sits on rd_data / weight_in; pulse load_weight
        edge_in();
        load_weight = 1'b1;
        tick();
        edge_in(); load_weight = 1'b0; tick();
    endtask

    // zero acts with act_in_valid high: clears the PE psum chain
    task automatic prime_array();
        edge_in();
        act_in_valid = 1'b1;
        for (int i = 0; i < ROW; i++) act_in[i] = '0;
        tick();
        for (int c = 0; c < PRIME; c++) begin edge_in(); tick(); end
        sb_start = cyc + 1;
    endtask

    // one UB line read per activation vector, broadcast to the ROW act inputs
    // on the next cycle; during round 0 the next weight tile is pushed into
    // the FIFO while the array computes (prefetch)
    task automatic stream_acts(input int r);
        for (int k = 0; k < K; k++) begin
            edge_in();
            if (k > 0)
                for (int i = 0; i < ROW; i++)
                    act_in[i] = ub_rd_data[i*ACT_WIDTH +: ACT_WIDTH];
            ub_rd_en   = 1'b1;
            ub_rd_addr = UB_AW'(k);
            if (r == 0 && k == 1) begin          // prefetch tile 2
                wf_wr_en   = 1'b1;
                wf_wr_data = pack_w(2);
            end else begin
                wf_wr_en   = 1'b0;
            end
            tick();
            if (k == 0) r0 = cyc;                // cycle that issued the read
        end
        // last vector: broadcast it, stop reading
        edge_in();
        for (int i = 0; i < ROW; i++)
            act_in[i] = ub_rd_data[i*ACT_WIDTH +: ACT_WIDTH];
        ub_rd_en   = 1'b0;
        ub_rd_addr = '0;
        wf_wr_en   = 1'b0;
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

    // (a) every cycle against the waveform model,
    // (b) every wavefront/column against dot(A[r][k][:], Wcur[:,j])
    task automatic check_round(input int r);
        int cc, acc, sb_err, gerr;
        int gemm_gold [0:K-1][0:COL-1];

        sb_err = 0;
        checks += (sb_end - sb_start + 1) * COL;
        for (int c = sb_start; c <= sb_end; c++) begin
            for (int j = 0; j < COL; j++) begin
                acc = 0;
                for (int i = 0; i < ROW; i++)
                    acc += Wcur[i][j] * $signed(act_hist[i][c + 1 - ROW - j]);
                if ($signed(psum_hist[j][c]) !== acc) begin
                    sb_err++;
                    if (sb_err <= 6)
                        $display("  [FAIL] round %0d cyc %0d col %0d: psum_out=%0d, weight/act model=%0d",
                                 r, c, j, $signed(psum_hist[j][c]), acc);
                end
            end
        end
        errors += sb_err;
        if (sb_err == 0)
            $display("  [PASS] %-46s (%0d cycles x %0d cols)",
                     "MXU follows the UB/WeightFIFO model",
                     sb_end - sb_start + 1, COL);

        gerr = 0;
        for (int k = 0; k < K; k++) begin
            for (int j = 0; j < COL; j++) begin
                acc = 0;
                for (int i = 0; i < ROW; i++) acc += Wcur[i][j] * A[r][k][i];
                gemm_gold[k][j] = acc;
                cc = r0 + k + ROW + j;           // dot product lands here
                checks++;
                if ($signed(psum_hist[j][cc]) !== acc || !pvalid_hist[cc]) begin
                    errors++; gerr++;
                    $display("  [FAIL] round %0d wavefront %0d col %0d: cyc %0d psum_out=%0d valid=%b, exp=%0d",
                             r, k, j, cc, $signed(psum_hist[j][cc]), pvalid_hist[cc], acc);
                end else begin
                    $display("  [PASS] round %0d wavefront %0d col %0d = %0d (cyc %0d, valid=1)",
                             r, k, j, acc, cc);
                end
            end
        end

        $display("       round %0d GEMM  A(%0d x %0d) * W(%0d x %0d):",
                 r, K, ROW, ROW, COL);
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

    // ---- Waveform dump / watchdog -------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "ub_mxu_wf_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, ub_mxu_wf_tb);
    end

    initial begin
        #(CLK_PERIOD * 100000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- Stimulus -----------------------------------------------------------
    initial begin
        // default inputs
        rst = 1'b0;
        ub_rd_en = 1'b0; ub_rd_addr = '0; ub_wr_en = 1'b0;
        ub_wr_addr = '0; ub_wr_data = '0;
        wf_wr_en = 1'b0; wf_wr_data = '0; wf_rd_en = 1'b0;
        load_weight = 1'b0; act_in_valid = 1'b0; psum_in_valid = 1'b0;
        for (int i = 0; i < ROW; i++) begin
            act_in[i] = '0;
            for (int j = 0; j < COL; j++) Wcur[i][j] = 0;
        end
        for (int j = 0; j < COL; j++) psum_in[j] = '0;

        // signed, round-dependent data (negative products included)
        for (int r = 0; r < NT; r++) begin
            for (int k = 0; k < K; k++)
                for (int i = 0; i < ROW; i++)
                    A[r][k][i] = (((r*11 + k*ROW + i) * 5) % 9) - 4;
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    W[r][i][j] = (((r*7 + i*COL + j) * 5) % 9) - 4;
        end

        $display("=============================================================");
        $display(" UB + MXU + WeightFIFO integration tb");
        $display("   UB : rtl/ub.sv  (WIDTH=%0d, WORD_SIZE=%0d -> %0d-bit act lines, DEPTH=%0d)",
                 ACT_WIDTH, ROW, ACT_LINE, UB_DEPTH);
        $display("   WF : rtl/weight_fifo.sv (TILE=%0d, WIDTH=%0d, DEPTH=%0d -> %0d-bit weight tiles)",
                 WF_TILES, WEIGHT_WIDTH, WT_WORDS, WF_LINE);
        $display("   MXU: rtl/mxu (ROW=%0d, COL=%0d, K=%0d per round, %0d rounds)",
                 ROW, COL, K, NT);
        $display("=============================================================");

        reset_all();
        check_bit("WF empty after reset", wf_empty, 1'b1);
        check_bit("WF not full after reset", wf_full, 1'b0);

        // ---- prefetch: two weight tiles into the 2-entry FIFO ---------------
        $display("[prefetch] push weight tiles 0 and 1 into the WeightFIFO");
        wf_push(0);
        check_bit("WF not empty after 1 push", wf_empty, 1'b0);
        check_bit("WF not full after 1 push", wf_full, 1'b0);
        wf_push(1);
        check_bit("WF full after 2 pushes", wf_full, 1'b1);

        for (int r = 0; r < NT; r++) begin
            $display("[round %0d] act image %0d -> UB, weight tile %0d -> MXU%s",
                     r, r, r, (r == 0) ? " (prefetch tile 2 during compute)" : "");
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++) Wcur[i][j] = W[r][i][j];

            ub_write_image(r);
            ub_readback(r);
            wf_pop_and_load(r);
            check_bit($sformatf("round %0d WF full low after the pop", r),
                      wf_full, 1'b0);
            prime_array();
            stream_acts(r);
            if (r == 0)
                check_bit("WF full again after the prefetch push", wf_full, 1'b1);
            check_round(r);
        end
        check_bit("WF empty after the last round", wf_empty, 1'b1);

        $display("=============================================================");
        $display(" UB/MXU/WeightFIFO tb summary: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display(" *** UB + MXU + WEIGHTFIFO TB PASSED ***");
        else
            $display(" *** UB + MXU + WEIGHTFIFO TB FAILED ***");
        $display("=============================================================");
        if (errors != 0) $fatal(1, "UB/MXU/WeightFIFO tb failed");
        $finish;
    end

endmodule

`default_nettype wire
