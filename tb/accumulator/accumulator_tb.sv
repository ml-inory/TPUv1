//=============================================================================
// Accumulator testbench
// DUT: rtl/accumulator.sv - double-buffered psum accumulator
//
// Behaviour pinned by this tb (from the DUT header):
//   * every psum_valid cycle stores the whole psum vector into block
//     block_idx, then block_idx advances 0,1,...,BLOCK_NUM-1,0,...  (wrap)
//   * dout is the combinational sum of the blocks of the OTHER group (the
//     half that is not being written), so while one K-segment is filled the
//     finished segment is readable; the final K-sum is the sum of the two
//     segment results (dout seen in the two phases)
//   * dout adds in WIDTH-bit precision (wraps on overflow, like the RTL)
//   * reset clears every block, the block pointer and dout
//
// Two instances:
//   * u_a: WIDTH=32, COL=4, BLOCK_NUM=4   (2 blocks per group)
//   * u_b: WIDTH=8,  COL=2, BLOCK_NUM=2   (minimal double buffer)
//
// Checked: reset, write addressing (every block is written in order and later
// overwritten after the wrap), read side (dout == sum of the opposite group,
// recomputed against a mirror every cycle), psum_valid=0 does nothing, and a
// 200-cycle random stream with periodic resets.
//
//   make accumulator_tb   -> runs the tb (self-checking)
//   +wave=<file>          VCD path (default accumulator_tb.vcd)
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module accumulator_tb;

    localparam int CLK_PERIOD = 10;

    // ---- instance A ---------------------------------------------------------
    localparam int WA = 32, CA = 4, BA = 4;
    // ---- instance B ---------------------------------------------------------
    localparam int WB = 8,  CB = 2, BB = 2;

    logic clk, rst;

    logic              va;
    logic [WA-1:0]     pa [0:CA-1];
    logic [WA-1:0]     da [0:CA-1];

    logic              vb;
    logic [WB-1:0]     pb [0:CB-1];
    logic [WB-1:0]     db [0:CB-1];

    Accumulator #(.WIDTH(WA), .COL(CA), .BLOCK_NUM(BA)) u_a (
        .clk(clk), .rst(rst), .psum_valid(va), .psum(pa), .dout(da)
    );
    Accumulator #(.WIDTH(WB), .COL(CB), .BLOCK_NUM(BB)) u_b (
        .clk(clk), .rst(rst), .psum_valid(vb), .psum(pb), .dout(db)
    );

    // ---- clock --------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- mirror models ------------------------------------------------------
    logic [WA-1:0] macc_a [0:BA-1][0:CA-1];
    int            mptr_a;
    logic [WB-1:0] macc_b [0:BB-1][0:CB-1];
    int            mptr_b;

    int errors = 0;
    int checks = 0;
    int cyc    = 0;

    // ---- helpers ------------------------------------------------------------
    task automatic check_int(input string name, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-52s got=%0d  exp=%0d", name, got, exp);
        end else begin
            $display("  [PASS] %-52s = %0d", name, got);
        end
    endtask

    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-52s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-52s = %b", name, got);
        end
    endtask

    // expected dout of instance A: sum of the group the write pointer is NOT in
    function automatic logic [WA-1:0] exp_dout_a(input int j);
        logic [WA-1:0] s;
        s = '0;
        for (int b = 0; b < BA/2; b++)
            s = (mptr_a >= BA/2) ? (s + macc_a[b][j]) : (s + macc_a[BA/2 + b][j]);
        exp_dout_a = s;
    endfunction

    function automatic logic [WB-1:0] exp_dout_b(input int j);
        logic [WB-1:0] s;
        s = '0;
        for (int b = 0; b < BB/2; b++)
            s = (mptr_b >= BB/2) ? (s + macc_b[b][j]) : (s + macc_b[BB/2 + b][j]);
        exp_dout_b = s;
    endfunction

    // compare the whole DUT state (pointer + every block + dout) with the mirror
    task automatic check_state_a(input string tag, input bit quiet);
        int snap;
        snap = errors;
        checks++;
        if (int'(u_a.block_idx) !== mptr_a) begin
            errors++;
            $display("  [FAIL] %-52s got=%0d exp=%0d", {tag, ": block_idx"},
                     u_a.block_idx, mptr_a);
        end
        for (int b = 0; b < BA; b++)
            for (int j = 0; j < CA; j++) begin
                checks++;
                if (u_a.acc[b][j] !== macc_a[b][j]) begin
                    errors++;
                    if (errors <= 30)
                        $display("  [FAIL] %-52s acc[%0d][%0d] got=%h exp=%h",
                                 tag, b, j, u_a.acc[b][j], macc_a[b][j]);
                end
            end
        for (int j = 0; j < CA; j++) begin
            checks++;
            if (u_a.dout[j] !== exp_dout_a(j)) begin
                errors++;
                if (errors <= 30)
                    $display("  [FAIL] %-52s dout[%0d] got=%h exp=%h [cyc %0d]",
                             tag, j, u_a.dout[j], exp_dout_a(j), cyc);
            end
        end
        if (errors == snap && !quiet)
            $display("  [PASS] %-52s (ptr=%0d, %0d blocks, %0d cols)",
                     tag, mptr_a, BA, CA);
    endtask

    task automatic check_state_b(input string tag, input bit quiet);
        int snap;
        snap = errors;
        checks++;
        for (int b = 0; b < BB; b++)
            for (int j = 0; j < CB; j++) begin
                checks++;
                if (u_b.acc[b][j] !== macc_b[b][j]) begin
                    errors++;
                    if (errors <= 30)
                        $display("  [FAIL] %-52s acc[%0d][%0d] got=%h exp=%h",
                                 tag, b, j, u_b.acc[b][j], macc_b[b][j]);
                end
            end
        for (int j = 0; j < CB; j++) begin
            checks++;
            if (u_b.dout[j] !== exp_dout_b(j)) begin
                errors++;
                if (errors <= 30)
                    $display("  [FAIL] %-52s dout[%0d] got=%h exp=%h [cyc %0d]",
                             tag, j, u_b.dout[j], exp_dout_b(j), cyc);
            end
        end
        if (errors == snap && !quiet)
            $display("  [PASS] %-52s", tag);
    endtask

    // enter the next clock cycle and let combinational logic settle
    task automatic new_cycle();
        @(negedge clk); #1;
        cyc++;
    endtask

    // mirror updates that a clock edge performs
    task automatic mirror_edge();
        if (va) begin
            for (int j = 0; j < CA; j++) macc_a[mptr_a][j] = pa[j];
            mptr_a = (mptr_a == BA-1) ? 0 : mptr_a + 1;
        end
        if (vb) begin
            for (int j = 0; j < CB; j++) macc_b[mptr_b][j] = pb[j];
            mptr_b = (mptr_b == BB-1) ? 0 : mptr_b + 1;
        end
    endtask

    // ---- waveform / watchdog -------------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "accumulator_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, accumulator_tb);
    end

    initial begin
        #(CLK_PERIOD * 100000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- stimulus ------------------------------------------------------------
    initial begin
        rst = 1'b1; va = 1'b0; vb = 1'b0;
        for (int j = 0; j < CA; j++) pa[j] = '0;
        for (int j = 0; j < CB; j++) pb[j] = '0;
        mptr_a = 0; mptr_b = 0;
        for (int b = 0; b < BA; b++) for (int j = 0; j < CA; j++) macc_a[b][j] = '0;
        for (int b = 0; b < BB; b++) for (int j = 0; j < CB; j++) macc_b[b][j] = '0;

        $display("===============================================================");
        $display(" Accumulator testbench");
        $display("   u_a: WIDTH=%0d COL=%0d BLOCK_NUM=%0d (%0d blocks per group)",
                 WA, CA, BA, BA/2);
        $display("   u_b: WIDTH=%0d COL=%0d BLOCK_NUM=%0d (%0d blocks per group)",
                 WB, CB, BB, BB/2);
        $display("===============================================================");

        // ---- [1] reset -------------------------------------------------------
        $display("[1] Reset clears blocks, pointer and dout");
        repeat (3) new_cycle();
        mirror_edge();
        rst = 1'b0;
        new_cycle();
        check_state_a("reset: state cleared", 1'b0);
        check_state_b("reset: state cleared (u_b)", 1'b0);
        check_int("reset: u_a block_idx", u_a.block_idx, 0);

        // ---- [2] directed writes: addressing + double-buffered readout --------
        $display("[2] Write %0d vectors (one per block) + %0d wrap-over writes",
                 BA, 2);
        for (int w = 0; w < BA + 2; w++) begin
            for (int j = 0; j < CA; j++) pa[j] = WA'((w*CA + j) * 7 + 1);
            for (int j = 0; j < CB; j++) pb[j] = WB'((w*CB + j) * 5 + 3);
            va = 1'b1; vb = 1'b1;
            new_cycle();                 // the write happens at this edge
            mirror_edge();
            va = 1'b0; vb = 1'b0;
            check_state_a($sformatf("after write %0d", w), 1'b0);
            check_state_b($sformatf("after write %0d (u_b)", w), 1'b0);
        end
        check_int("u_a: pointer wrapped back", u_a.block_idx, (BA + 2) % BA);

        // ---- [3] psum_valid low does nothing ---------------------------------
        $display("[3] psum_valid=0 holds the state");
        new_cycle(); mirror_edge();
        new_cycle(); mirror_edge();
        check_state_a("idle cycles: state unchanged", 1'b0);

        // ---- [4] two phases -> the two segment sums --------------------------
        // fill the pointer's group first, then read the finished group
        $display("[4] Segment readout: dout shows the finished group");
        for (int w = 0; w < BA/2; w++) begin
            for (int j = 0; j < CA; j++) pa[j] = WA'(32'h1000 + w*CA + j);
            va = 1'b1;
            new_cycle(); mirror_edge();
            va = 1'b0;
            check_state_a($sformatf("segment write %0d", w), 1'b0);
        end

        // ---- [5] randomized stream -------------------------------------------
        $display("[5] Randomized stream (200 cycles) vs the mirror model");
        for (int c = 0; c < 200; c++) begin
            if (c % 61 == 0) begin          // periodic reset, valid may be high
                va = $urandom_range(0, 1);
                vb = $urandom_range(0, 1);
                for (int j = 0; j < CA; j++) pa[j] = WA'($urandom);
                for (int j = 0; j < CB; j++) pb[j] = WB'($urandom);
                new_cycle();
                rst = 1'b1;
                mptr_a = 0; mptr_b = 0;
                for (int b = 0; b < BA; b++) for (int j = 0; j < CA; j++) macc_a[b][j] = '0;
                for (int b = 0; b < BB; b++) for (int j = 0; j < CB; j++) macc_b[b][j] = '0;
                new_cycle();
                rst = 1'b0;
                va = 1'b0; vb = 1'b0;
            end else begin
                va = ($urandom_range(0, 2) != 0);
                vb = ($urandom_range(0, 2) != 0);
                for (int j = 0; j < CA; j++) pa[j] = WA'($urandom);
                for (int j = 0; j < CB; j++) pb[j] = WB'($urandom);
                new_cycle();
                mirror_edge();
                va = 1'b0; vb = 1'b0;
            end
            // compare quietly (failures print inside, capped)
            check_state_a("random", 1'b1);
        end

        // ---- summary ---------------------------------------------------------
        $display("===============================================================");
        $display(" Accumulator tb summary: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display(" *** ACCUMULATOR TB PASSED ***");
        else
            $display(" *** ACCUMULATOR TB FAILED ***");
        $display("===============================================================");
        if (errors != 0) $fatal(1, "Accumulator tb failed");
        $finish;
    end

endmodule

`default_nettype wire
