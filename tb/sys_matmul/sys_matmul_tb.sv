//=============================================================================
// System level matmul verification
// DUTs: rtl/ub.sv + rtl/weight_fifo.sv + rtl/weight_reshape.sv +
//       rtl/mxu/{mxu,pe,delay_chain}.sv + rtl/mxu_controller.sv +
//       rtl/accumulator.sv   (the whole data path, controller driven)
//
// The tb runs several matmul camps through the real RTL, dumps the random
// inputs and the observed results to a trace, and sys_matmul_tb.py recomputes
// the expected numbers and compares element by element.
//
// Accumulation model (matches rtl/accumulator.sv):
//   * one "pass" feeds PASS_K activation vectors; the MXU streams one complete
//     wavefront per cycle (aligned psum_out + psum_out_valid) and the
//     accumulator writes one block per wavefront, so after PASS_K writes its
//     group is full and dout is that pass's partial sum
//         out[j] = sum_k sum_i A[k][i] * W[i][j]
//   * BLOCK_NUM = 2*PASS_K, so consecutive passes write opposite groups: the
//     previous pass result stays readable while the next pass accumulates
//     (double buffered role switching)
//   * a batch with K_TOTAL > PASS_K is split into passes; the final matmul
//     result is the sum of the per-pass results (what the Python model checks)
//
// Campaigns:
//   1) K_TOTAL = PASS_K              -> single pass, basic correctness
//   2) K_TOTAL = 3*PASS_K            -> multi pass, cross pass accumulation
//   3) 2 batches x K_TOTAL=2*PASS_K  -> double buffer role switching
//
//   make sys_matmul_tb   -> vvp + Python reference check
//   +trace=<file>        trace path (default sys_matmul_trace.txt)
//   +wave=<file>         VCD path   (default sys_matmul_tb.vcd)
//   +seed=<n>            RNG seed for the random matrices
//
// Trace format ('#' header carries the shape):
//   B <batch> <K_TOTAL>
//   W <batch> <i> <j> <value>                    weight element (signed)
//   A <batch> <k> <i> <value>                    activation element (signed)
//   R <batch> <pass> <k0> <n> <col0..colCOL-1>   dout after that pass
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module sys_matmul_tb;

    localparam int ROW          = 4;
    localparam int COL          = 3;
    localparam int ACT_WIDTH    = 8;
    localparam int WEIGHT_WIDTH = 8;
    localparam int ACC_WIDTH    = 32;

    localparam int PASS_K       = 4;              // acts per pass
    localparam int ACC_BLOCKS   = 2*PASS_K;       // double buffered groups
    localparam int MAXK         = 4*PASS_K;       // deepest K in this tb
    localparam int N_BATCHES    = 4;              // batches across campaigns

    localparam int ACT_LINE     = ACT_WIDTH*ROW;
    localparam int UB_DEPTH     = 64;
    localparam int UB_AW        = $clog2(UB_DEPTH);

    localparam int WT_WORDS     = ROW*COL;
    localparam int WF_LINE      = WEIGHT_WIDTH*WT_WORDS;
    localparam int WF_TILES     = 2;

    localparam int CLK_PERIOD   = 10;

    // controller states
    localparam logic [2:0] S_IDLE = 3'd0, S_LOAD = 3'd1, S_COMP = 3'd2,
                           S_DRAIN = 3'd3, S_DONE = 3'd4;

    logic clk, rst, start;

    // UB
    wire                  ub_rd_en;
    wire [UB_AW-1:0]      ub_rd_addr;
    logic [ACT_LINE-1:0]  ub_rd_data;
    logic                 ub_wr_en;
    logic [UB_AW-1:0]     ub_wr_addr;
    logic [ACT_LINE-1:0]  ub_wr_data;

    // WeightFIFO
    logic                 wf_wr_en;
    logic [WF_LINE-1:0]   wf_wr_data;
    logic                 wf_full;
    wire                  wf_rd_en;
    logic [WF_LINE-1:0]   wf_rd_data;
    logic                 wf_empty;

    // controller
    logic                 ctrl_load_weight, ctrl_weight_rd_en;
    logic                 ctrl_act_rd_en, ctrl_act_in_valid, ctrl_done;
    logic [15:0]          act_cycles;

    // MXU ports
    logic [WEIGHT_WIDTH-1:0]  weight_in [0:ROW-1][0:COL-1];
    logic [ACT_WIDTH-1:0]     act_in    [0:ROW-1];
    logic                     psum_in_valid;
    logic [ACC_WIDTH-1:0]     psum_in   [0:COL-1];
    logic                     psum_out_valid;
    logic [ACC_WIDTH-1:0]     psum_out  [0:COL-1];

    // accumulator
    logic [ACC_WIDTH-1:0]     acc_psum  [0:COL-1];
    logic                     acc_valid;
    logic [ACC_WIDTH-1:0]     acc_dout  [0:COL-1];

    // act address glue
    logic [UB_AW-1:0] act_addr;
    logic [UB_AW-1:0] tb_rd_addr;
    logic             tb_rd_en;
    bit               preread_done;
    wire              preread_en;

    assign preread_en = (u_ctrl.cur_state == S_LOAD) && !preread_done;
    assign ub_rd_en   = ctrl_act_rd_en | preread_en | tb_rd_en;
    assign ub_rd_addr = tb_rd_en ? tb_rd_addr
                                 : (preread_en ? UB_AW'(0) : act_addr);
    assign wf_rd_en   = ctrl_weight_rd_en;

    // one UB read per activation vector is issued one cycle ahead of its use,
    // and vector 0 is pre-read during LOAD_WEIGHT
    always_ff @(posedge clk) begin
        if (rst) begin
            act_addr     <= '0;
            preread_done <= 1'b0;
        end else begin
            if (u_ctrl.cur_state == S_IDLE) begin
                act_addr     <= '0;
                preread_done <= 1'b0;
            end else if (preread_en) begin
                preread_done <= 1'b1;
                act_addr     <= UB_AW'(1);
            end else if (ctrl_act_rd_en && u_ctrl.cur_state == S_COMP) begin
                act_addr <= act_addr + 1'b1;
            end
        end
    end

    // UB line -> MXU act ports, WeightReshape lanes -> MXU weight ports
    always_comb begin
        for (int i = 0; i < ROW; i++) act_in[i] = ub_rd_data[i*ACT_WIDTH +: ACT_WIDTH];
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                weight_in[i][j] = u_reshape.dout[i][j];
    end

    // ---- DUTs ---------------------------------------------------------------
    UB #(.WIDTH(ACT_WIDTH), .WORD_SIZE(ROW), .DEPTH(UB_DEPTH)) u_ub (
        .clk(clk), .rst(rst),
        .rd_en(ub_rd_en), .rd_addr(ub_rd_addr), .rd_data(ub_rd_data),
        .wr_en(ub_wr_en), .wr_addr(ub_wr_addr), .wr_data(ub_wr_data)
    );

    WeightFIFO #(.TILE(WF_TILES), .WIDTH(WEIGHT_WIDTH), .DEPTH(WT_WORDS)) u_wf (
        .clk(clk), .rst(rst),
        .wr_en(wf_wr_en), .wr_data(wf_wr_data), .full(wf_full),
        .rd_en(wf_rd_en), .rd_data(wf_rd_data), .empty(wf_empty)
    );

    WeightReshape #(.WIDTH(WEIGHT_WIDTH), .ROW(ROW), .COL(COL)) u_reshape (
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

    Accumulator #(.WIDTH(ACC_WIDTH), .COL(COL), .BLOCK_NUM(ACC_BLOCKS)) u_acc (
        .clk(clk), .rst(rst),
        .psum_valid(acc_valid), .psum(acc_psum), .dout(acc_dout)
    );

    // accumulator feed (Icarus does not propagate unpacked array output ports)
    initial begin
        for (int j = 0; j < COL; j++) acc_psum[j] = '0;
        acc_valid = 1'b0;
        forever begin
            @(negedge clk);
            for (int j = 0; j < COL; j++) acc_psum[j] = u_mxu.psum_out[j];
            acc_valid = u_mxu.psum_out_valid;
        end
    end

    // ---- clock --------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- random matrices / bookkeeping --------------------------------------
    int  A [0:N_BATCHES-1][0:MAXK-1][0:ROW-1];   // activation vectors
    int  W [0:N_BATCHES-1][0:ROW-1][0:COL-1];    // weights
    int  ktotal [0:N_BATCHES-1];                 // accumulation depth per batch

    int  checks = 0;
    int  errors = 0;
    int  cyc    = 0;
    int  guard;
    bit  done_seen;
    integer fd;

    function automatic logic [ACT_LINE-1:0] act_line(input int b, input int k);
        logic [ACT_LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++)
            l[i*ACT_WIDTH +: ACT_WIDTH] = ACT_WIDTH'(A[b][k][i]);
        act_line = l;
    endfunction

    function automatic logic [ACT_LINE-1:0] sentinel_line();
        logic [ACT_LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++) l[i*ACT_WIDTH +: ACT_WIDTH] = 8'hA5;
        sentinel_line = l;
    endfunction

    function automatic logic [WF_LINE-1:0] pack_w(input int b);
        logic [WF_LINE-1:0] l;
        l = '0;
        for (int i = 0; i < ROW; i++)
            for (int j = 0; j < COL; j++)
                l[(i*COL + j)*WEIGHT_WIDTH +: WEIGHT_WIDTH] = WEIGHT_WIDTH'(W[b][i][j]);
        pack_w = l;
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

    task automatic new_cycle();
        @(negedge clk); #1;
        cyc++;
    endtask

    // fresh start for one batch: the accumulator's write pointer returns to a
    // group boundary, so every pass below fills exactly one double buffer group
    // (without this a batch whose pass count is not a multiple of PASS_K would
    // leave the pointer mid group and the next batch's first dout would be the
    // stale group - see the notes in the report)
    task automatic reset_system();
        rst = 1'b1;
        repeat (3) new_cycle();
        rst = 1'b0;
        new_cycle();
    endtask

    // write one pass worth of activation vectors into UB (line k = vector k,
    // plus the 0xA5 sentinel line at index n that the one-ahead read touches)
    task automatic ub_write_pass(input int b, input int k0, input int n);
        for (int a = 0; a <= n; a++) begin
            ub_wr_en   = 1'b1;
            ub_wr_addr = UB_AW'(a);
            ub_wr_data = (a < n) ? act_line(b, k0 + a) : sentinel_line();
            new_cycle();
        end
        ub_wr_en = 1'b0;
    endtask

    task automatic push_tile(input int b);
        wf_wr_en   = 1'b1;
        wf_wr_data = pack_w(b);
        new_cycle();
        wf_wr_en   = 1'b0;
    endtask

    // run one pass: reload the weight tile, refill UB, start the controller,
    // wait for done and dump the accumulated result of that pass
    task automatic run_pass(input int b, input int pass, input int k0, input int n,
                            input string label);
        push_tile(b);
        ub_write_pass(b, k0, n);

        act_cycles = 16'(n);
        start = 1'b1;
        new_cycle();
        start = 1'b0;

        done_seen = 1'b0; guard = 0;
        while (!done_seen && guard < 500) begin
            if (ctrl_done) done_seen = 1'b1;
            else begin new_cycle(); guard++; end
        end
        check_bit($sformatf("%s pass %0d: controller done", label, pass),
                  done_seen, 1'b1);
        check_bit($sformatf("%s pass %0d: psum_out_valid low at done", label, pass),
                  psum_out_valid, 1'b0);

        // the accumulator writes finish with the last wavefront; give them a
        // couple of cycles before reading the group result
        new_cycle(); new_cycle();
        $fwrite(fd, "R %0d %0d %0d %0d", b, pass, k0, n);
        for (int j = 0; j < COL; j++) $fwrite(fd, " %0d", $signed(u_acc.dout[j]));
        $fwrite(fd, "\n");
    endtask

    // ---- waveform / watchdog -------------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "sys_matmul_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, sys_matmul_tb);
    end

    initial begin
        #(CLK_PERIOD * 200000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- stimulus ------------------------------------------------------------
    initial begin
        string trace_file;
        int    seed;
        int    batch;

        if (!$value$plusargs("trace=%s", trace_file))
            trace_file = "sys_matmul_trace.txt";
        if (!$value$plusargs("seed=%d", seed)) seed = 1;
        void'($urandom(seed));

        fd = $fopen(trace_file, "w");
        if (fd == 0) $fatal(1, "cannot open trace file '%s'", trace_file);
        $fwrite(fd, "# SYSMAT ROW=%0d COL=%0d PASS_K=%0d BLOCK_NUM=%0d\n",
                ROW, COL, PASS_K, ACC_BLOCKS);

        // defaults
        rst = 1'b1; start = 1'b0; act_cycles = 16'(PASS_K);
        ub_wr_en = 1'b0; ub_wr_addr = '0; ub_wr_data = '0;
        tb_rd_en = 1'b0; tb_rd_addr = '0;
        wf_wr_en = 1'b0; wf_wr_data = '0;
        psum_in_valid = 1'b0;
        for (int j = 0; j < COL; j++) psum_in[j] = '0;

        $display("===============================================================");
        $display(" system level matmul verification (seed=%0d)", seed);
        $display("   ROW=%0d COL=%0d  PASS_K=%0d  accumulator BLOCK_NUM=%0d",
                 ROW, COL, PASS_K, ACC_BLOCKS);
        $display("   trace -> %s   (reference model: sys_matmul_tb.py)", trace_file);
        $display("===============================================================");

        repeat (3) new_cycle();
        rst = 1'b0;
        new_cycle();

        // random matrices (also dumped into the trace for the Python model)
        for (int b = 0; b < N_BATCHES; b++) begin
            ktotal[b] = (b == 0) ? PASS_K : (b == 1) ? 3*PASS_K : 2*PASS_K;
            for (int k = 0; k < MAXK; k++)
                for (int i = 0; i < ROW; i++)
                    A[b][k][i] = $urandom_range(0, 15) - 8;
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    W[b][i][j] = $urandom_range(0, 15) - 8;
        end
        for (int b = 0; b < N_BATCHES; b++) begin
            $fwrite(fd, "B %0d %0d\n", b, ktotal[b]);
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    $fwrite(fd, "W %0d %0d %0d %0d\n", b, i, j, W[b][i][j]);
            for (int k = 0; k < ktotal[b]; k++)
                for (int i = 0; i < ROW; i++)
                    $fwrite(fd, "A %0d %0d %0d %0d\n", b, k, i, A[b][k][i]);
        end

        // ---- campaign 1: single pass (K = PASS_K) ---------------------------
        batch = 0;
        $display("[campaign 1] single pass: K_TOTAL=%0d (= PASS_K)", ktotal[batch]);
        reset_system();
        run_pass(batch, 0, 0, PASS_K, "single pass");

        // ---- campaign 2: multi pass (K = 3*PASS_K) --------------------------
        batch = 1;
        $display("[campaign 2] multi pass: K_TOTAL=%0d (%0d passes)",
                 ktotal[batch], ktotal[batch]/PASS_K);
        reset_system();
        for (int p = 0; p < ktotal[batch]/PASS_K; p++)
            run_pass(batch, p, p*PASS_K, PASS_K, "multi pass");

        // ---- campaign 3: two batches (double buffer role switching) ---------
        $display("[campaign 3] two batches x K_TOTAL=%0d", 2*PASS_K);
        for (int b = 2; b < N_BATCHES; b++) begin
            reset_system();
            for (int p = 0; p < ktotal[b]/PASS_K; p++)
                run_pass(b, p, p*PASS_K, PASS_K, "batch");
        end

        $fclose(fd);
        $display("===============================================================");
        $display(" sys_matmul tb: %0d protocol checks, %0d failures (data check: sys_matmul_tb.py)",
                 checks, errors);
        $display("===============================================================");
        if (errors != 0) $fatal(1, "system matmul tb protocol checks failed");
        $finish;
    end

endmodule

`default_nettype wire
