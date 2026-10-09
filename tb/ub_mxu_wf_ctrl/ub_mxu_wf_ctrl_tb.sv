//=============================================================================
// UB + MXU + WeightFIFO + WeightReshape + MXU_Controller system testbench
//
// This is the full controller-driven data path of the TPU:
//   * acts   : UB (one line = one ROW-wide activation vector) -> MXU act ports
//   * weights: WeightFIFO -> WeightReshape -> MXU weight ports
//   * control: rtl/mxu_controller.sv runs IDLE -> LOAD_WEIGHT -> COMPUTE ->
//              DRAIN -> DONE and drives load_weight / weight_rd_en / act_rd_en
//              / act_in_valid, consuming weight_fifo_empty and psum_out_valid
//
// The tb only orchestrates: it writes each round's activation image into UB
// (and checks the read-back), prefetches weight tiles into the FIFO, pulses
// start, and then verifies the MXU results against A_round x W_tile.
//
// Glue that a real top level needs and the tb models here:
//   * UB read data is registered, so the activation address runs one ahead:
//     vector 0 is pre-read during LOAD_WEIGHT, then during COMPUTE cycle c the
//     address c+1 is issued (its data is consumed in cycle c+1) while the MXU
//     consumes the vector that was read in the previous cycle.
//   * the last COMPUTE cycle reads one don't-care line (index K); that line is
//     filled with the 0xA5 sentinel so nothing is X.
//   * WeightReshape's unpacked output port cannot drive a tb-level array in
//     Icarus, so its lanes are copied into weight_in through the hierarchy
//     (in real RTL it is a plain wire: .weight_in(u_reshape.dout)).
//
// Checked per round: UB image, reshaped weight lanes, the whole controller
// protocol (COMPUTE length, one load pulse, done after psum_out_valid drops)
// and psum_out[j] == dot(A[k][:], W[:,j]) at cycle c0 + k + ROW + COL - 1, where c0
// is the first COMPUTE cycle (interval convention: a value sampled right after
// the falling edge is the one that lives in that cycle).
//
//   make ub_mxu_wf_ctrl_tb   -> runs the tb (self-checking)
//   +wave=<file>             VCD path (default ub_mxu_wf_ctrl_tb.vcd)
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module ub_mxu_wf_ctrl_tb;

    localparam int ROW          = 4;
    localparam int COL          = 3;
    localparam int K            = 4;      // act vectors per round
    localparam int NT           = 3;      // rounds / weight tiles
    localparam int ACT_WIDTH    = 8;
    localparam int WEIGHT_WIDTH = 8;
    localparam int ACC_WIDTH    = 32;

    localparam int ACT_LINE     = ACT_WIDTH*ROW;
    localparam int UB_DEPTH     = 64;
    localparam int UB_AW        = $clog2(UB_DEPTH);

    localparam int WT_WORDS     = ROW*COL;
    localparam int WF_LINE      = WEIGHT_WIDTH*WT_WORDS;
    localparam int WF_TILES     = 2;

    localparam int CLK_PERIOD   = 10;
    localparam int MAXC         = 600;

    // controller states (mirrors rtl/mxu_controller.sv)
    localparam logic [2:0] S_IDLE = 3'd0, S_LOAD = 3'd1, S_COMP = 3'd2,
                           S_DRAIN = 3'd3, S_DONE = 3'd4;

    logic clk, rst, start;

    // ---- UB ----------------------------------------------------------------
    wire                  ub_rd_en;         // driven by a continuous assign
    wire [UB_AW-1:0]      ub_rd_addr;       // driven by a continuous assign
    logic [ACT_LINE-1:0]  ub_rd_data;
    logic                 ub_wr_en;
    logic [UB_AW-1:0]     ub_wr_addr;
    logic [ACT_LINE-1:0]  ub_wr_data;

    // ---- WeightFIFO --------------------------------------------------------
    logic                 wf_wr_en;
    logic [WF_LINE-1:0]   wf_wr_data;
    logic                 wf_full;
    wire                  wf_rd_en;         // driven by a continuous assign
    logic [WF_LINE-1:0]   wf_rd_data;
    logic                 wf_empty;

    // ---- controller --------------------------------------------------------
    logic                 ctrl_load_weight;
    logic                 ctrl_weight_rd_en;
    logic                 ctrl_act_rd_en;
    logic                 ctrl_act_in_valid;
    logic                 ctrl_done;
    logic [15:0]          act_cycles;

    // ---- MXU ---------------------------------------------------------------
    logic [WEIGHT_WIDTH-1:0]  weight_in [0:ROW-1][0:COL-1];
    logic [ACT_WIDTH-1:0]     act_in    [0:ROW-1];
    logic                     psum_in_valid;
    logic [ACC_WIDTH-1:0]     psum_in   [0:COL-1];
    logic                     psum_out_valid;
    logic [ACC_WIDTH-1:0]     psum_out  [0:COL-1];

    // ---- act address glue --------------------------------------------------
    logic [UB_AW-1:0] act_addr;
    bit               preread_done;
    wire              preread_en;
    logic             tb_rd_en;             // tb-driven UB read (image checking)
    logic [UB_AW-1:0] tb_rd_addr;

    assign preread_en = (u_ctrl.cur_state == S_LOAD) && !preread_done;
    assign ub_rd_en   = ctrl_act_rd_en | preread_en | tb_rd_en;
    assign ub_rd_addr = tb_rd_en ? tb_rd_addr
                                 : (preread_en ? UB_AW'(0) : act_addr);
    assign wf_rd_en   = ctrl_weight_rd_en;

    always_ff @(posedge clk) begin
        if (rst) begin
            act_addr     <= '0;
            preread_done <= 1'b0;
        end else begin
            if (u_ctrl.cur_state == S_IDLE) begin
                act_addr     <= '0;             // fresh round
                preread_done <= 1'b0;
            end else if (preread_en) begin
                preread_done <= 1'b1;           // vector 0 is in the UB pipeline
                act_addr     <= UB_AW'(1);      // COMPUTE cycle 0 must read vector 1
            end else if (ctrl_act_rd_en && u_ctrl.cur_state == S_COMP) begin
                act_addr <= act_addr + 1'b1;
            end
        end
    end

    // UB line -> MXU act ports
    always_comb begin
        for (int i = 0; i < ROW; i++)
            act_in[i] = ub_rd_data[i*ACT_WIDTH +: ACT_WIDTH];
    end

    // WeightReshape lanes -> MXU weight ports
    always_comb begin
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                weight_in[i][j] = u_reshape.dout[i][j];
    end

    // ---- DUTs --------------------------------------------------------------
    UB #(
        .WIDTH(ACT_WIDTH), .WORD_SIZE(ROW), .DEPTH(UB_DEPTH)
    ) u_ub (
        .clk(clk), .rst(rst),
        .rd_en(ub_rd_en), .rd_addr(ub_rd_addr), .rd_data(ub_rd_data),
        .wr_en(ub_wr_en), .wr_addr(ub_wr_addr), .wr_data(ub_wr_data)
    );

    WeightFIFO #(
        .TILE(WF_TILES), .WIDTH(WEIGHT_WIDTH), .DEPTH(WT_WORDS)
    ) u_wf (
        .clk(clk), .rst(rst),
        .wr_en(wf_wr_en), .wr_data(wf_wr_data), .full(wf_full),
        .rd_en(wf_rd_en), .rd_data(wf_rd_data), .empty(wf_empty)
    );

    WeightReshape #(
        .WIDTH(WEIGHT_WIDTH), .ROW(ROW), .COL(COL)
    ) u_reshape (
        .din(wf_rd_data), .dout()
    );

    MXU #(
        .ROW(ROW), .COL(COL),
        .ACT_WIDTH(ACT_WIDTH), .WEIGHT_WIDTH(WEIGHT_WIDTH), .ACC_WIDTH(ACC_WIDTH)
    ) u_mxu (
        .clk(clk), .rst(rst),
        .load_weight(ctrl_load_weight), .weight_in(weight_in),
        .act_in_valid(ctrl_act_in_valid), .act_in(act_in),
        .psum_in_valid(psum_in_valid), .psum_in(psum_in),
        .psum_out_valid(psum_out_valid), .psum_out(psum_out)
    );

    MXU_Controller u_ctrl (
        .clk(clk), .rst(rst), .start(start),
        .load_weight(ctrl_load_weight),
        .weight_rd_en(ctrl_weight_rd_en),
        .weight_fifo_empty(wf_empty),
        .act_rd_en(ctrl_act_rd_en),
        .act_in_valid(ctrl_act_in_valid),
        .act_cycles(act_cycles),
        .psum_out_valid(psum_out_valid),
        .done(ctrl_done)
    );

    // ---- clock / reset ------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- golden data / bookkeeping -----------------------------------------
    int  A [0:NT-1][0:K-1][0:ROW-1];
    int  W [0:NT-1][0:ROW-1][0:COL-1];
    int  cyc = 0;
    int  checks = 0;
    int  errors = 0;
    int  errs0;

    logic [ACC_WIDTH-1:0] psum_hist [0:COL-1][0:MAXC];
    bit                   pv_hist   [0:MAXC];
    logic [2:0]           st_hist   [0:MAXC];

    // ---- helpers -----------------------------------------------------------
    function automatic logic [ACT_LINE-1:0] act_line(input int r, input int k);
        logic [ACT_LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++)
            l[i*ACT_WIDTH +: ACT_WIDTH] = ACT_WIDTH'(A[r][k][i]);
        act_line = l;
    endfunction

    function automatic logic [ACT_LINE-1:0] sentinel_line();
        logic [ACT_LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++) l[i*ACT_WIDTH +: ACT_WIDTH] = 8'hA5;
        sentinel_line = l;
    endfunction

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

    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-50s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-50s = %b", name, got);
        end
    endtask

    task automatic check_int(input string name, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-50s got=%0d  exp=%0d", name, got, exp);
        end else begin
            $display("  [PASS] %-50s = %0d", name, got);
        end
    endtask

    task automatic check_acts(input string name, input logic [ACT_LINE-1:0] got,
                              input logic [ACT_LINE-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-50s got=%h exp=%h (first bad row %0d)",
                     name, got, exp, bad_lane(got, exp));
        end else begin
            $display("  [PASS] %-50s = %h", name, got);
        end
    endtask

    // enter the next clock cycle and let combinational logic settle
    task automatic new_cycle();
        @(negedge clk); #1;
        cyc++;
    endtask

    // one-cycle WeightFIFO push (call between edges; cleared automatically)
    task automatic push_tile(input int t);
        wf_wr_en   = 1'b1;
        wf_wr_data = pack_w(t);
        new_cycle();
        wf_wr_en   = 1'b0;
    endtask

    // write the round's act image (+ sentinels) into UB
    task automatic ub_write_image(input int r);
        for (int a = 0; a < UB_DEPTH; a++) begin
            ub_wr_en   = 1'b1;
            ub_wr_addr = UB_AW'(a);
            if (a < K)                             ub_wr_data = act_line(r, a);
            else if (a == K || a == UB_DEPTH-1)     ub_wr_data = sentinel_line();
            else                                    ub_wr_data = '0;
            new_cycle();
        end
        ub_wr_en = 1'b0;
    endtask

    task automatic ub_readback(input int r);
        errs0 = errors;
        tb_rd_en = 1'b1;
        for (int a = 0; a < K; a++) begin
            tb_rd_addr = UB_AW'(a);
            new_cycle();
            check_acts($sformatf("round %0d UB line[%0d]", r, a), ub_rd_data, act_line(r, a));
        end
        for (int a = K; a < UB_DEPTH; a += UB_DEPTH-1-K) begin
            tb_rd_addr = UB_AW'(a);
            new_cycle();
            check_acts($sformatf("round %0d UB sentinel[%0d]", r, a), ub_rd_data, sentinel_line());
        end
        tb_rd_en = 1'b0;
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-50s (%0d lines + 2 sentinels)", "UB holds the act image", K);
    endtask

    // ---- run one round through the controller -------------------------------
    int  c0, n_comp, guard, n_loads_hist;
    bit  done_seen;

    task automatic run_round(input int r);
        // start
        start = 1'b1;
        new_cycle();                         // IDLE -> LOAD_WEIGHT
        start = 1'b0;

        c0 = -1; n_comp = 0; done_seen = 1'b0; guard = 0;
        while (!done_seen && guard < 400) begin
            // observe the cycle we are in
            for (int j = 0; j < COL; j++) psum_hist[j][cyc] = u_mxu.psum_out[j];
            pv_hist[cyc] = psum_out_valid;
            st_hist[cyc] = u_ctrl.cur_state;

            if (u_ctrl.cur_state == S_COMP) begin
                if (c0 < 0) c0 = cyc;
                n_comp++;
                if (n_comp == 1) begin          // prefetch tile 2 during round 0
                    if (r == 0) begin
                        wf_wr_en   = 1'b1;
                        wf_wr_data = pack_w(2);
                    end
                end
            end
            if (ctrl_done) done_seen = 1'b1;
            else begin
                new_cycle(); guard++;
                wf_wr_en = 1'b0;
            end
        end
        if (!done_seen) begin
            errors++;
            $display("  [FAIL] round %0d: controller never signalled done", r);
        end else begin
            $display("       round %0d: COMPUTE starts at cyc %0d, done at cyc %0d (%0d compute cycles)",
                     r, c0, cyc, n_comp);
            check_bit($sformatf("round %0d: done is a DONE-state pulse", r),
                      (ctrl_done && u_ctrl.cur_state == S_DONE), 1'b1);
            check_int($sformatf("round %0d: COMPUTE length", r), n_comp, K);
            check_bit($sformatf("round %0d: psum_out_valid low at done", r),
                      psum_out_valid, 1'b0);
        end
    endtask

    // ---- result checks for one round ----------------------------------------
    task automatic check_round_results(input int r);
        int acc, gerr;
        gerr = 0;
        // reshaped weight lanes must match this round's tile
        checks++;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                if (u_reshape.dout[i][j] !== WEIGHT_WIDTH'(W[r][i][j])) begin
                    errors++; gerr++;
                    if (gerr <= 6)
                        $display("  [FAIL] round %0d WeightReshape lane[%0d][%0d] got=%02h exp=%02h",
                                 r, i, j, u_reshape.dout[i][j], WEIGHT_WIDTH'(W[r][i][j]));
                end
        if (gerr == 0)
            $display("  [PASS] %-50s (%0d lanes)", $sformatf("round %0d reshaped weight tile", r), ROW*COL);

        // psum_out[j] at cycle c0 + k + ROW + COL - 1 == dot(A[k][:], W[:,j])
        gerr = 0;
        for (int k = 0; k < K; k++) begin
            for (int j = 0; j < COL; j++) begin
                acc = 0;
                for (int i = 0; i < ROW; i++) acc += W[r][i][j] * A[r][k][i];
                checks++;
                if ($signed(psum_hist[j][c0 + k + ROW + COL - 1]) !== acc) begin
                    errors++; gerr++;
                    $display("  [FAIL] round %0d wavefront %0d col %0d (cyc %0d): psum_out=%0d exp=%0d",
                             r, k, j, c0 + k + ROW + COL - 1,
                             $signed(psum_hist[j][c0 + k + ROW + COL - 1]), acc);
                end else begin
                    $display("  [PASS] round %0d wavefront %0d col %0d = %0d (cyc %0d)",
                             r, k, j, acc, c0 + k + ROW + COL - 1);
                end
            end
        end
    endtask

    // ---- waveform / watchdog -------------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "ub_mxu_wf_ctrl_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, ub_mxu_wf_ctrl_tb);
    end

    initial begin
        #(CLK_PERIOD * 200000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- stimulus ------------------------------------------------------------
    initial begin
        // defaults
        rst = 1'b1; start = 1'b0; act_cycles = 16'(K);
        ub_wr_en = 1'b0; ub_wr_addr = '0; ub_wr_data = '0;
        wf_wr_en = 1'b0; wf_wr_data = '0;
        psum_in_valid = 1'b0;
        for (int j = 0; j < COL; j++) psum_in[j] = '0;

        // signed, round-dependent data
        for (int r = 0; r < NT; r++) begin
            for (int k = 0; k < K; k++)
                for (int i = 0; i < ROW; i++)
                    A[r][k][i] = (((r*11 + k*ROW + i) * 5) % 9) - 4;
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    W[r][i][j] = (((r*7 + i*COL + j) * 5) % 9) - 4;
        end

        $display("===============================================================");
        $display(" UB + MXU + WeightFIFO + WeightReshape + MXU_Controller system tb");
        $display("   ROW=%0d COL=%0d K=%0d rounds=%0d, act_cycles=%0d", ROW, COL, K, NT, K);
        $display("   UB line = %0d bits, WeightFIFO entry = %0d bits, TILE=%0d",
                 ACT_LINE, WF_LINE, WF_TILES);
        $display("===============================================================");

        // reset everything (only once: the FIFO must keep its prefetched tiles)
        repeat (3) new_cycle();
        rst = 1'b0;
        new_cycle();
        check_bit("WF empty after reset", wf_empty, 1'b1);

        // prefetch the first two tiles (FIFO depth is 2 -> full)
        push_tile(0);
        check_bit("WF not empty after 1st push", wf_empty, 1'b0);
        push_tile(1);
        check_bit("WF full after 2nd push", wf_full, 1'b1);

        for (int r = 0; r < NT; r++) begin
            $display("[round %0d] act image %0d -> UB, weight tile %0d -> MXU%s",
                     r, r, r, (r == 0) ? " (tile 2 prefetched during COMPUTE)" : "");
            ub_write_image(r);
            ub_readback(r);
            run_round(r);
            check_bit($sformatf("round %0d: WF not full after the pop", r),
                      wf_full, (r == 0) ? 1'b1 : 1'b0);
            check_round_results(r);
            new_cycle();
            check_bit($sformatf("round %0d: back to IDLE", r),
                      (u_ctrl.cur_state == S_IDLE), 1'b1);
            check_bit($sformatf("round %0d: done deasserted in IDLE", r), ctrl_done, 1'b0);
        end

        check_bit("WF empty after the last round", wf_empty, 1'b1);

        $display("===============================================================");
        $display(" system tb summary: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display(" *** UB + MXU + WEIGHTFIFO + CONTROLLER TB PASSED ***");
        else
            $display(" *** UB + MXU + WEIGHTFIFO + CONTROLLER TB FAILED ***");
        $display("===============================================================");
        if (errors != 0) $fatal(1, "system tb failed");
        $finish;
    end

endmodule

`default_nettype wire
