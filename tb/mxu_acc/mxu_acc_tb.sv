//=============================================================================
// MXU + Accumulator integration testbench (controller driven)
//
// DUTs: rtl/mxu/{mxu,pe,delay_chain}.sv + rtl/accumulator.sv +
//       rtl/mxu_controller.sv
//
// Flow per segment r: load weight tile W_r, run the controller (COMPUTE feeds
// K activation vectors, one per cycle), let the accumulator capture the MXU
// psum stream, then read the K-accumulated result.
//
// rtl/mxu now aligns its own psum outputs (column j delayed by COL-1-j) and
// psum_out_valid marks the K cycles where all COL psums of one wavefront are
// on the bus, so the accumulator can be wired straight to the MXU.
//
// BLOCK_NUM is 2*K, so the accumulator is used as the double buffer of the
// design: one group of K blocks per segment, and while segment r+1 is being
// accumulated dout still shows segment r's result.
//
// Checked per segment: controller protocol (COMPUTE length, done), the K
// aligned captures (each block == dot(A[m][:], W[:,j]) per column) and
// dout == sum_k dot(A[k][:], W[:,j]); plus the double-buffered readout.
//
//   make mxu_acc_tb   -> runs the tb (self-checking)
//   +wave=<file>      VCD path (default mxu_acc_tb.vcd)
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module mxu_acc_tb;

    localparam int ROW   = 4;
    localparam int COL   = 3;
    localparam int K     = 4;              // act vectors (accumulation length)
    localparam int NSEG  = 3;              // segments / weight tiles
    localparam int AW    = 8;
    localparam int WW    = 8;
    localparam int ACCW  = 32;
    localparam int BLOCK_NUM = 2*K;        // double buffer: K blocks per segment
    localparam int CLK_PERIOD = 10;

    localparam logic [2:0] S_IDLE = 3'd0, S_LOAD = 3'd1, S_COMP = 3'd2,
                           S_DRAIN = 3'd3, S_DONE = 3'd4;

    logic clk, rst, start;
    logic done, c_load_weight, c_weight_rd_en, c_act_rd_en, c_act_in_valid;
    logic weight_fifo_empty;
    logic [15:0] act_cycles;

    logic [WW-1:0]  weight_in [0:ROW-1][0:COL-1];
    logic [AW-1:0]  act_in    [0:ROW-1];
    logic           psum_in_valid;
    logic [ACCW-1:0] psum_in  [0:COL-1];
    logic           psum_out_valid;
    logic [ACCW-1:0] psum_out [0:COL-1];

    logic [ACCW-1:0] acc_psum  [0:COL-1];   // aligned psum into the accumulator
    logic            acc_valid;
    logic [ACCW-1:0] acc_dout  [0:COL-1];

    // ---- golden data --------------------------------------------------------
    int A [0:NSEG-1][0:K-1][0:ROW-1];
    int W [0:NSEG-1][0:ROW-1][0:COL-1];
    int seg;                 // current segment
    int act_idx;             // activation vector currently on act_in

    // ---- DUTs ---------------------------------------------------------------
    MXU #(
        .ROW(ROW), .COL(COL),
        .ACT_WIDTH(AW), .WEIGHT_WIDTH(WW), .ACC_WIDTH(ACCW)
    ) u_mxu (
        .clk(clk), .rst(rst),
        .load_weight(c_load_weight), .weight_in(weight_in),
        .act_in_valid(c_act_in_valid), .act_in(act_in),
        .psum_in_valid(psum_in_valid), .psum_in(psum_in),
        .psum_out_valid(psum_out_valid), .psum_out(psum_out)
    );

    Accumulator #(
        .WIDTH(ACCW), .COL(COL), .BLOCK_NUM(BLOCK_NUM)
    ) u_acc (
        .clk(clk), .rst(rst),
        .psum_valid(acc_valid), .psum(acc_psum), .dout(acc_dout)
    );

    MXU_Controller u_ctrl (
        .clk(clk), .rst(rst), .start(start),
        .load_weight(c_load_weight),
        .weight_rd_en(c_weight_rd_en),
        .weight_fifo_empty(weight_fifo_empty),
        .act_rd_en(c_act_rd_en),
        .act_in_valid(c_act_in_valid),
        .act_cycles(act_cycles),
        .psum_out_valid(psum_out_valid),
        .done(done)
    );

    // activation index: advances once per COMPUTE cycle, cleared outside
    always_ff @(posedge clk) begin
        if (rst)
            act_idx <= 0;
        else if (u_ctrl.cur_state == S_COMP)
            act_idx <= act_idx + 1;
        else
            act_idx <= 0;
    end

    // ---- accumulator feed + stimulus glue ----------------------------------
    // Icarus does not propagate an unpacked-array output port to a tb-level
    // array, so the MXU psum outputs are copied through the hierarchy; the copy
    // runs in a falling-edge loop (blocking assignments there are equivalent to
    // a wire for the accumulator, which samples on the rising edge).
    initial begin
        for (int j = 0; j < COL; j++) acc_psum[j] = '0;
        acc_valid = 1'b0;
        forever begin
            @(negedge clk);
            for (int j = 0; j < COL; j++) acc_psum[j] = u_mxu.psum_out[j];
            acc_valid = u_mxu.psum_out_valid;
            // weight tile + activation vector of the current segment
            for (int i = 0; i < ROW; i++) begin
                act_in[i] = AW'(A[seg][act_idx][i]);
                for (int j = 0; j < COL; j++)
                    weight_in[i][j] = WW'(W[seg][i][j]);
            end
        end
    end

    // ---- clock --------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- bookkeeping --------------------------------------------------------
    int errors = 0;
    int checks = 0;
    int cyc    = 0;
    int guard;
    bit done_seen;

    function automatic int dot(input int s, input int k, input int j);
        int acc;
        acc = 0;
        for (int i = 0; i < ROW; i++) acc += W[s][i][j] * A[s][k][i];
        dot = acc;
    endfunction

    function automatic int ksum(input int s, input int j);
        int acc;
        acc = 0;
        for (int k = 0; k < K; k++) acc += dot(s, k, j);
        ksum = acc;
    endfunction

    task automatic check_int(input string name, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-52s got=%0d  exp=%0d", name, got, exp);
        end else begin
            $display("  [PASS] %-52s = %0d", name, got);
        end
    endtask

    task automatic new_cycle();
        @(negedge clk); #1;
        cyc++;
    endtask

    //---- waveform / watchdog -------------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "mxu_acc_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, mxu_acc_tb);
    end

    initial begin
        #(CLK_PERIOD * 100000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- stimulus ------------------------------------------------------------
    initial begin
        rst = 1'b1; start = 1'b0; act_cycles = 16'(K);
        weight_fifo_empty = 1'b0;      // weights come from the tb (no FIFO here)
        psum_in_valid = 1'b0;
        for (int j = 0; j < COL; j++) psum_in[j] = '0;
        seg = 0; act_idx = 0;

        // signed, segment-dependent data
        for (int s = 0; s < NSEG; s++) begin
            for (int k = 0; k < K; k++)
                for (int i = 0; i < ROW; i++)
                    A[s][k][i] = (((s*11 + k*ROW + i) * 5) % 9) - 4;
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    W[s][i][j] = (((s*7 + i*COL + j) * 5) % 9) - 4;
        end

        $display("===============================================================");
        $display(" MXU + Accumulator integration tb (controller driven)");
        $display("   ROW=%0d COL=%0d K=%0d segments=%0d, accumulator BLOCK_NUM=%0d",
                 ROW, COL, K, NSEG, BLOCK_NUM);
        $display("   alignment: psum col j delayed %0d..0, capture window delayed %0d",
                 COL-1, ROW+COL-1);
        $display("===============================================================");

        repeat (3) new_cycle();
        rst = 1'b0;
        new_cycle();

        for (int s = 0; s < NSEG; s++) begin
            seg = s;
            $display("[segment %0d] load W%0d, compute A%0d x W%0d, accumulate K=%0d",
                     s, s, s, s, K);

            // start the controller
            start = 1'b1;
            new_cycle();
            start = 1'b0;

            // while this segment computes, dout must still show the previous
            // segment's result (double buffered readout)
            guard = 0;
            while (u_ctrl.cur_state != S_COMP && guard < 20) begin
                new_cycle(); guard++;
            end
            if (s > 0)
                for (int j = 0; j < COL; j++)
                    check_int($sformatf("segment %0d result held during segment %0d",
                                        s-1, s),
                              $signed(u_acc.dout[j]), ksum(s-1, j));

            // wait for done
            done_seen = 1'b0; guard = 0;
            while (!done_seen && guard < 200) begin
                if (done) done_seen = 1'b1;
                else begin new_cycle(); guard++; end
            end
            check_int($sformatf("segment %0d: done seen", s), done_seen, 1);

            // the captures finish a couple of cycles after done
            new_cycle(); new_cycle();

            // (a) every captured block must hold one aligned wavefront
            for (int m = 0; m < K; m++)
                for (int j = 0; j < COL; j++)
                    check_int($sformatf("segment %0d block %0d col %0d = wavefront %0d",
                                        s, (s*K + m) % BLOCK_NUM, j, m),
                              $signed(u_acc.acc[(s*K + m) % BLOCK_NUM][j]),
                              dot(s, m, j));

            // (b) dout = the K-accumulated result of this segment
            for (int j = 0; j < COL; j++)
                check_int($sformatf("segment %0d dout[%0d] == sum_k dot", s, j),
                          $signed(u_acc.dout[j]), ksum(s, j));
        end

        $display("===============================================================");
        $display(" MXU+Accumulator tb summary: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display(" *** MXU + ACCUMULATOR TB PASSED ***");
        else
            $display(" *** MXU + ACCUMULATOR TB FAILED ***");
        $display("===============================================================");
        if (errors != 0) $fatal(1, "MXU/Accumulator tb failed");
        $finish;
    end

endmodule

`default_nettype wire
