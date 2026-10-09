//=============================================================================
// MXU_Controller testbench
// DUT: rtl/mxu_controller.sv
//      IDLE -> LOAD_WEIGHT -> COMPUTE -> DRAIN -> DONE -> IDLE
//
// The tb drives the controller with a behavioural model of the datapath it
// talks to, so the protocol can be checked cycle by cycle:
//   * mock WeightFIFO: empty flag + a tile queue. The read data is REGISTERED
//     (rtl/weight_fifo.sv pops on rd_en and presents the entry from the next
//     edge on), so a popped tile is only visible to the MXU the cycle AFTER
//     weight_rd_en.
//   * mock MXU: latches whatever is on its weight port on every load_weight
//     pulse (that is what rtl/mxu/pe.sv does) and drives psum_out_valid as
//     act_in_valid delayed by ROW-1 cycles. That window was measured on the
//     real rtl/mxu (ROW=4, COL=3): psum_out_valid is high from
//     first_act + ROW-1 to last_act + ROW-1.
//
// Timing discipline: `new_cycle()` enters the next clock cycle (falling edge)
// and lets the combinational logic settle, so every observation below belongs
// to the cycle it is checked in.
//
// Contract checked:
//   [1] IDLE: all outputs low, stays there without start
//   [2] start -> LOAD_WEIGHT: waits while the FIFO is empty, then the MXU must
//       hold EXACTLY the popped tile when COMPUTE starts (start held high
//       while busy is ignored)
//   [3] COMPUTE: act_rd_en/act_in_valid high for exactly act_cycles cycles,
//       load_weight/weight_rd_en low
//   [4] DRAIN: no new acts, and DONE must not be entered while
//       psum_out_valid is still high (results still streaming)
//   [5] DONE -> IDLE with all outputs low, controller restartable
//   [6] act_cycles = 1 boundary
//   [7] reset in the middle of COMPUTE -> IDLE, act counter cleared, a fresh
//       run consumes exactly act_cycles acts again
//
//   make mxu_controller_tb   -> runs the tb (self-checking)
//   +wave=<file>             VCD path (default mxu_controller_tb.vcd)
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module mxu_controller_tb;

    localparam int CLK_PERIOD = 10;
    localparam int ROW        = 4;      // MXU shape used for the valid latency
    localparam int COL        = 3;
    localparam int LAT        = ROW - 1;   // measured psum_out_valid latency
    localparam int NCYCLES    = 4;      // default act_cycles
    localparam int TILE_W     = 8;      // mock weight tile data

    // controller state encoding, mirrors rtl/mxu_controller.sv
    localparam logic [2:0] S_IDLE = 3'd0, S_LOAD = 3'd1, S_COMP = 3'd2,
                           S_DRAIN = 3'd3, S_DONE = 3'd4;

    logic clk, rst, start;
    logic load_weight, weight_rd_en, act_rd_en, act_in_valid;
    logic weight_fifo_empty;
    logic [15:0] act_cycles;
    logic psum_out_valid;
    logic done;

    MXU_Controller dut (
        .clk               (clk),
        .rst               (rst),
        .start             (start),
        .load_weight       (load_weight),
        .weight_rd_en      (weight_rd_en),
        .weight_fifo_empty (weight_fifo_empty),
        .act_rd_en         (act_rd_en),
        .act_in_valid      (act_in_valid),
        .act_cycles        (act_cycles),
        .psum_out_valid    (psum_out_valid),
        .done              (done)
    );

    // ---- clock --------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- mock WeightFIFO: queue + registered read data ----------------------
    logic [TILE_W-1:0] wq [0:7];
    int                wq_head, wq_cnt;
    logic [TILE_W-1:0] mock_rd_data;      // registered FIFO output
    logic [TILE_W-1:0] latched;           // tile the (mock) MXU loaded
    int                n_loads;

    assign weight_fifo_empty = (wq_cnt == 0);

    // ---- mock MXU: psum_out_valid = act_in_valid delayed by LAT -------------
    logic [LAT-1:0] valid_sr;
    assign psum_out_valid = valid_sr[LAT-1];

    always_ff @(posedge clk) begin
        if (rst) begin
            wq_head      <= 0;
            wq_cnt       <= 0;
            mock_rd_data <= '0;
            latched      <= '0;
            n_loads      <= 0;
            valid_sr     <= '0;
        end else begin
            valid_sr <= {valid_sr[LAT-2:0], act_in_valid};
            if (weight_rd_en && !weight_fifo_empty) begin
                mock_rd_data <= wq[wq_head];      // registered pop
                wq_head      <= (wq_head == 7) ? 0 : wq_head + 1;
                wq_cnt       <= wq_cnt - 1;
            end
            if (load_weight) begin                // MXU samples weight_in here
                latched <= mock_rd_data;
                n_loads <= n_loads + 1;
            end
        end
    end

    // ---- bookkeeping / scratch ---------------------------------------------
    int  errors = 0;
    int  checks = 0;
    int  cyc    = 0;
    int  guard, n_comp, n_drain, n_loads0;
    bit  bad, premature, saw_load;

    // ---- helpers ------------------------------------------------------------
    function automatic string st_name(input logic [2:0] s);
        case (s)
            S_IDLE:  st_name = "IDLE";
            S_LOAD:  st_name = "LOAD_WEIGHT";
            S_COMP:  st_name = "COMPUTE";
            S_DRAIN: st_name = "DRAIN";
            S_DONE:  st_name = "DONE";
            default: st_name = "?";
        endcase
    endfunction

    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-52s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-52s = %b", name, got);
        end
    endtask

    task automatic check_int(input string name, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-52s got=%0d  exp=%0d", name, got, exp);
        end else begin
            $display("  [PASS] %-52s = %0d", name, got);
        end
    endtask

    task automatic check_state(input string name, input logic [2:0] exp);
        checks++;
        if (dut.cur_state !== exp) begin
            errors++;
            $display("  [FAIL] %-52s got=%s  exp=%s", name,
                     st_name(dut.cur_state), st_name(exp));
        end else begin
            $display("  [PASS] %-52s = %s", name, st_name(exp));
        end
    endtask

    task automatic check_tile(input string name, input logic [TILE_W-1:0] got,
                              input logic [TILE_W-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-52s got=%02h  exp=%02h", name, got, exp);
        end else begin
            $display("  [PASS] %-52s = %02h", name, got);
        end
    endtask

    // enter the next clock cycle and let the combinational logic settle
    task automatic new_cycle();
        @(negedge clk); #1;
        cyc++;
    endtask

    // push a weight tile into the mock FIFO (visible from the current cycle on)
    task automatic push_tile(input logic [TILE_W-1:0] t);
        wq[(wq_head + wq_cnt) % 8] = t;
        wq_cnt = wq_cnt + 1;
    endtask

    // wait until the controller leaves state s (bounded)
    task automatic wait_leave(input logic [2:0] s);
        guard = 0;
        while (dut.cur_state == s && guard < 300) begin
            new_cycle(); guard++;
        end
        if (guard >= 300) begin
            errors++;
            $display("  [FAIL] controller stuck in %s", st_name(s));
        end
    endtask

    // ---- waveform / watchdog -------------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "mxu_controller_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, mxu_controller_tb);
    end

    initial begin
        #(CLK_PERIOD * 100000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- stimulus ------------------------------------------------------------
    initial begin
        rst = 1'b1; start = 1'b0; act_cycles = 16'(NCYCLES);
        wq_cnt = 0; wq_head = 0;

        $display("=============================================================");
        $display(" MXU_Controller testbench (mock datapath)");
        $display("   psum_out_valid fill-latency model = %0d cycles", LAT);
        $display("=============================================================");

        // ---- [1] reset / IDLE ------------------------------------------------
        $display("[1] Reset and IDLE");
        repeat (3) new_cycle();
        rst = 1'b0;
        new_cycle();
        check_state("IDLE after reset", S_IDLE);
        check_bit("IDLE: load_weight low", load_weight, 1'b0);
        check_bit("IDLE: weight_rd_en low", weight_rd_en, 1'b0);
        check_bit("IDLE: act_rd_en low", act_rd_en, 1'b0);
        check_bit("IDLE: act_in_valid low", act_in_valid, 1'b0);
        new_cycle();
        check_state("IDLE holds while start stays low", S_IDLE);

        // ---- [2] start, wait for the weight, load it -------------------------
        $display("[2] LOAD_WEIGHT: wait while the WeightFIFO is empty, then load");
        start = 1'b1;
        new_cycle();
        check_state("start -> LOAD_WEIGHT", S_LOAD);
        check_bit("LOAD_WEIGHT: 1st cycle pops (rd_en high, load low)",
                  (weight_rd_en & ~load_weight), 1'b1);
        check_bit("LOAD_WEIGHT: act_rd_en low", act_rd_en, 1'b0);
        check_bit("LOAD_WEIGHT: act_in_valid low", act_in_valid, 1'b0);

        // start is only sampled in IDLE: holding it here must change nothing
        new_cycle(); new_cycle();
        check_state("holds in LOAD_WEIGHT while the FIFO is empty", S_LOAD);
        check_int("no weight latched while the FIFO is empty", n_loads, 0);

        start = 1'b0;
        push_tile(8'hA1);                 // tile visible from this cycle on
        n_loads0 = n_loads;
        saw_load = 1'b0;
        guard = 0;
        while (dut.cur_state == S_LOAD && guard < 300) begin
            if (load_weight) begin
                saw_load = 1'b1;
                check_bit("load phase: weight_rd_en low", weight_rd_en, 1'b0);
                check_bit("load phase: popped tile sits on rd_data",
                          (mock_rd_data === 8'hA1), 1'b1);
            end
            new_cycle(); guard++;
        end
        check_state("FIFO tile available -> COMPUTE", S_COMP);
        check_bit("a load pulse happened", saw_load, 1'b1);
        check_tile("MXU latched exactly the popped tile", latched, 8'hA1);
        check_int("exactly one weight load pulse", n_loads - n_loads0, 1);

        // ---- [3] COMPUTE -----------------------------------------------------
        $display("[3] COMPUTE: exactly %0d act cycles, nothing else driven", NCYCLES);
        n_comp = 0; bad = 1'b0;
        while (dut.cur_state == S_COMP && n_comp < 100) begin
            if (act_in_valid !== 1'b1 || act_rd_en !== 1'b1)    bad = 1'b1;
            if (load_weight  !== 1'b0 || weight_rd_en !== 1'b0) bad = 1'b1;
            n_comp++;
            new_cycle();
        end
        check_int("COMPUTE cycles with act_in_valid/act_rd_en", n_comp, NCYCLES);
        check_bit("COMPUTE: act held, load/weight_rd low", bad, 1'b0);
        check_bit("COMPUTE: done low", done, 1'b0);
        check_state("COMPUTE -> DRAIN", S_DRAIN);

        // ---- [4] DRAIN -------------------------------------------------------
        $display("[4] DRAIN: must wait until psum_out_valid has dropped");
        check_bit("DRAIN: done low", done, 1'b0);
        n_drain = 0; premature = 1'b0;
        while (dut.cur_state == S_DRAIN && n_drain < 100) begin
            if (n_drain == 0 && psum_out_valid)
                $display("       (note: psum_out_valid is already high when DRAIN starts)");
            n_drain++;
            new_cycle();
            if (dut.cur_state == S_DONE && psum_out_valid) premature = 1'b1;
        end
        check_bit("DRAIN: DONE only after psum_out_valid drops", premature, 1'b0);
        check_state("reached DONE", S_DONE);
        $display("       (DRAIN lasted %0d cycles, fill-latency model is %0d)",
                 n_drain, LAT);

        // ---- [5] DONE -> IDLE ------------------------------------------------
        $display("[5] DONE -> IDLE, restartable");
        check_bit("DONE: load_weight low", load_weight, 1'b0);
        check_bit("DONE: weight_rd_en low", weight_rd_en, 1'b0);
        check_bit("DONE: act_rd_en low", act_rd_en, 1'b0);
        check_bit("DONE: act_in_valid low", act_in_valid, 1'b0);
        check_bit("DONE: done pulse high", done, 1'b1);
        new_cycle();
        check_state("DONE -> IDLE", S_IDLE);
        check_bit("back in IDLE with all outputs low",
                  (load_weight | weight_rd_en | act_rd_en | act_in_valid), 1'b0);
        check_bit("done low outside DONE", done, 1'b0);

        // ---- [6] act_cycles = 1 boundary -------------------------------------
        $display("[6] act_cycles = 1 boundary");
        act_cycles = 16'd1; start = 1'b1;
        new_cycle();
        check_state("act_cycles=1: start -> LOAD_WEIGHT", S_LOAD);
        start = 1'b0;
        push_tile(8'hB2);
        wait_leave(S_LOAD);
        check_state("act_cycles=1: -> COMPUTE", S_COMP);
        check_tile("act_cycles=1: MXU latched the tile", latched, 8'hB2);
        n_comp = 0; bad = 1'b0;
        while (dut.cur_state == S_COMP && n_comp < 100) begin
            if (act_in_valid !== 1'b1) bad = 1'b1;
            n_comp++;
            new_cycle();
        end
        check_int("act_cycles=1: COMPUTE cycles", n_comp, 1);
        check_bit("act_cycles=1: act_in_valid held", bad, 1'b0);
        premature = 1'b0;
        while (dut.cur_state == S_DRAIN && cyc < 600) begin
            new_cycle();
            if (dut.cur_state == S_DONE && psum_out_valid) premature = 1'b1;
        end
        check_bit("act_cycles=1: DRAIN waits for psum_out_valid", premature, 1'b0);
        wait_leave(S_DONE);
        check_state("act_cycles=1: back to IDLE", S_IDLE);

        // ---- [7] reset in the middle of COMPUTE ------------------------------
        $display("[7] Reset while COMPUTE, then a fresh full run");
        act_cycles = 16'(NCYCLES); start = 1'b1;
        new_cycle();
        start = 1'b0;
        push_tile(8'hC3);
        wait_leave(S_LOAD);
        check_state("reset test: in COMPUTE", S_COMP);
        new_cycle(); new_cycle();
        check_state("reset test: still in COMPUTE", S_COMP);
        rst = 1'b1;
        new_cycle(); new_cycle();
        check_state("reset -> IDLE", S_IDLE);
        check_bit("reset: outputs low",
                  (load_weight | weight_rd_en | act_rd_en | act_in_valid), 1'b0);
        rst = 1'b0; start = 1'b1;
        new_cycle();
        check_state("fresh start after reset -> LOAD_WEIGHT", S_LOAD);
        start = 1'b0;
        push_tile(8'hD4);
        wait_leave(S_LOAD);
        check_state("fresh run -> COMPUTE", S_COMP);
        check_tile("fresh run: MXU latched the tile", latched, 8'hD4);
        n_comp = 0;
        while (dut.cur_state == S_COMP && n_comp < 100) begin
            n_comp++;
            new_cycle();
        end
        check_int("fresh run: COMPUTE cycles (act counter was cleared)", n_comp, NCYCLES);
        premature = 1'b0;
        while (dut.cur_state == S_DRAIN && cyc < 900) begin
            new_cycle();
            if (dut.cur_state == S_DONE && psum_out_valid) premature = 1'b1;
        end
        check_bit("fresh run: DRAIN waits for psum_out_valid", premature, 1'b0);
        wait_leave(S_DONE);
        check_state("fresh run: back to IDLE", S_IDLE);

        // ---- summary ---------------------------------------------------------
        $display("=============================================================");
        $display(" MXU_Controller tb summary: %0d checks, %0d failures", checks, errors);
        if (errors == 0)
            $display(" *** MXU CONTROLLER TB PASSED ***");
        else
            $display(" *** MXU CONTROLLER TB FAILED ***");
        $display("=============================================================");
        if (errors != 0) $fatal(1, "MXU_Controller tb failed");
        $finish;
    end

endmodule

`default_nettype wire
