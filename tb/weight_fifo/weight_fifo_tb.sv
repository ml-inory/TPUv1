//=============================================================================
// WeightFIFO testbench
// DUT: rtl/weight_fifo.sv - weight prefetch/buffer FIFO
//
// One FIFO entry is a whole weight line of WIDTH*DEPTH bits (packed), TILE
// entries deep. Contract checked here (what a FIFO has to do):
//   * rst            -> empty = 1, full = 0, all entries cleared
//   * wr_en, ~full   -> wr_data is enqueued at the tail
//   * rd_en, ~empty  -> the head is dequeued and appears on rd_data during
//                       that same clock (rd_data is registered with it)
//   * rd_en + wr_en in the same clock: BOTH take effect, the length is
//     unchanged and the order is preserved (new element goes to the tail)
//   * rd_en while empty / wr_en while full are no-ops; a write request while
//     full is ignored even when a read runs in the same clock (conservative
//     full handling - this is pinned by check [3])
//   * full/empty are combinational from the number of entries
//
// Two instances: the main one (TILE=4, WIDTH=8, DEPTH=8 -> 64-bit lines) with
// a per-cycle mirror model and a Python reference check over the dumped trace
// (tb/weight_fifo/weight_fifo_tb.py), plus a single-entry instance (TILE=1)
// for the full == 1-entry corner.
//
//   make weight_fifo_tb        -> vvp + Python reference check
//   +trace=<file>              trace path (default weight_fifo_trace.txt)
//   +wave=<file>               VCD path   (default weight_fifo_tb.vcd)
//
// Trace format (one line per clock; '#' header carries the parameters):
//   cyc rst wr_en rd_en full empty wr_data rd_data        (hex)
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module weight_fifo_tb;

    localparam int WIDTH      = 8;
    localparam int DEPTH      = 8;      // words per weight line
    localparam int LINE       = WIDTH*DEPTH;
    localparam int TILE       = 4;      // entries in the FIFO
    localparam int CLK_PERIOD = 10;     // ns
    localparam int RUN_CYCLES = 400;

    localparam int T1 = 1;              // single-entry corner instance
    localparam int W1 = 8;
    localparam int D1 = 1;
    localparam int L1 = W1*D1;

    // ---- DUT signals --------------------------------------------------------
    logic clk, rst;

    logic             wr_en;
    logic [LINE-1:0]  wr_data;
    logic             full;
    logic             rd_en;
    logic [LINE-1:0]  rd_data;
    logic             empty;

    logic             wr_en1;
    logic [L1-1:0]    wr_data1;
    logic             full1;
    logic             rd_en1;
    logic [L1-1:0]    rd_data1;
    logic             empty1;

    // ---- DUTs ---------------------------------------------------------------
    WeightFIFO #(
        .TILE(TILE), .WIDTH(WIDTH), .DEPTH(DEPTH)
    ) dut (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (wr_en),
        .wr_data (wr_data),
        .full    (full),
        .rd_en   (rd_en),
        .rd_data (rd_data),
        .empty   (empty)
    );

    WeightFIFO #(
        .TILE(T1), .WIDTH(W1), .DEPTH(D1)
    ) dut1 (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (wr_en1),
        .wr_data (wr_data1),
        .full    (full1),
        .rd_en   (rd_en1),
        .rd_data (rd_data1),
        .empty   (empty1)
    );

    // ---- Clock --------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- Bookkeeping / mirror model ----------------------------------------
    int errors         = 0;
    int checks         = 0;
    int cycles_checked = 0;
    int errs0;
    int cyc = 0;
    integer fd;

    logic [LINE-1:0] model [0:TILE-1];   // mirror of the FIFO contents
    int              msize;              // number of valid entries
    logic [LINE-1:0] rd_exp;             // expected rd_data (X before 1st pop)
    logic [LINE-1:0] scratch;            // hold checks / drain expectation
    bit              mfull, mempty;

    // line pattern: every word gets its own value, so a word swap shows up
    function automatic logic [LINE-1:0] pat(input int a);
        logic [LINE-1:0] p;
        p = '0;
        for (int w = 0; w < DEPTH; w++)
            p[w*WIDTH +: WIDTH] = WIDTH'((a*DEPTH + w) * 37 + 11);
        pat = p;
    endfunction

    // index of the first word that differs (-1 when the lines match)
    function automatic int bad_word(input logic [LINE-1:0] got,
                                    input logic [LINE-1:0] exp);
        bad_word = -1;
        for (int w = 0; w < DEPTH; w++)
            if (bad_word < 0 && got[w*WIDTH +: WIDTH] !== exp[w*WIDTH +: WIDTH])
                bad_word = w;
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

    task automatic check_val(input string name, input logic [LINE-1:0] got,
                             input logic [LINE-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-46s got=%h exp=%h (first bad word %0d)",
                     name, got, exp, bad_word(got, exp));
        end else begin
            $display("  [PASS] %-46s = %h", name, got);
        end
    endtask

    task automatic check_val1(input string name, input logic [L1-1:0] got,
                              input logic [L1-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-46s got=%h exp=%h", name, got, exp);
        end else begin
            $display("  [PASS] %-46s = %h", name, got);
        end
    endtask

    // ---- Trace dump ---------------------------------------------------------
    task automatic dump();
        $fwrite(fd, "%0d %0d %0d %0d %0d %0d %h %h\n",
                cyc, rst, wr_en, rd_en, full, empty, wr_data, rd_data);
    endtask

    // drive rst at the falling edge, advance one clock, then sample
    // (the mirror implements the FIFO contract listed in the header)
    task automatic step(input logic r);
        bit rd_fire, wr_fire;
        @(negedge clk);
        rst = r;
        if (r) begin
            msize = 0;
            for (int q = 0; q < TILE; q++) model[q] = '0;
        end else begin
            rd_fire = rd_en && (msize > 0);
            wr_fire = wr_en && (msize < TILE);
            if (rd_fire) rd_exp = model[0];       // head -> rd_data this clock
            if (rd_fire) begin
                for (int q = 0; q < msize-1; q++) model[q] = model[q+1];
                msize--;
            end
            if (wr_fire) begin
                model[msize] = wr_data;           // tail (after a pop's shift)
                msize++;
            end
        end
        mfull  = (msize == TILE);
        mempty = (msize == 0);

        @(posedge clk); #1;
        cyc++;
        dump();
        cycles_checked++;
        if (rd_data !== rd_exp) begin
            errors++;
            if (errors <= 20)
                $display("  [FAIL] cyc %0d rd_data got=%h exp=%h (first bad word %0d) (rst=%b rd_en=%b wr_en=%b)",
                         cyc, rd_data, rd_exp, bad_word(rd_data, rd_exp),
                         rst, rd_en, wr_en);
        end
        if (full !== mfull || empty !== mempty) begin
            errors++;
            if (errors <= 20)
                $display("  [FAIL] cyc %0d flags got full=%b empty=%b exp full=%b empty=%b (size=%0d)",
                         cyc, full, empty, mfull, mempty, msize);
        end
    endtask

    // ---- Waveform dump / watchdog -------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "weight_fifo_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, weight_fifo_tb);
    end

    initial begin
        #(CLK_PERIOD * 100000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- Stimulus -----------------------------------------------------------
    initial begin
        string trace_file;
        if (!$value$plusargs("trace=%s", trace_file))
            trace_file = "weight_fifo_trace.txt";
        fd = $fopen(trace_file, "w");
        if (fd == 0)
            $fatal(1, "cannot open trace file '%s'", trace_file);
        $fwrite(fd, "# WF TILE=%0d WIDTH=%0d DEPTH=%0d\n", TILE, WIDTH, DEPTH);

        // input defaults
        rst = 1'b0;
        wr_en = 1'b0; wr_data = '0;
        rd_en = 1'b0;
        wr_en1 = 1'b0; wr_data1 = '0; rd_en1 = 1'b0;

        $display("=============================================================");
        $display(" WeightFIFO testbench");
        $display("   main : TILE=%0d, WIDTH=%0d words/line, DEPTH=%0d -> %0d-bit entries",
                 TILE, WIDTH, DEPTH, LINE);
        $display("   small: TILE=%0d, WIDTH=%0d, DEPTH=%0d (single-entry corner)",
                 T1, W1, D1);
        $display("   trace -> %s  (reference model: weight_fifo_tb.py)", trace_file);
        $display("=============================================================");

        // ---- [1] Reset ------------------------------------------------------
        $display("[1] Reset: empty, not full, rd while empty is a no-op");
        step(1'b1);
        check_bit("empty after reset", empty, 1'b1);
        check_bit("full after reset",  full,  1'b0);
        rd_en = 1'b1;
        step(1'b0);
        check_bit("empty stays set while rd_en on empty FIFO", empty, 1'b1);
        rd_en = 1'b0;
        step(1'b0);

        // ---- [2] Fill, flags, FIFO order, drain ------------------------------
        $display("[2] Fill %0d entries, check full/empty, pop them back in order",
                 TILE);
        for (int a = 0; a < TILE; a++) begin
            wr_en = 1'b1; wr_data = pat(a);
            step(1'b0);
            check_bit($sformatf("empty low after push %0d", a), empty, 1'b0);
        end
        wr_en = 1'b0;
        check_bit("full after the last push", full, 1'b1);
        step(1'b0);
        check_bit("full holds while the FIFO is full", full, 1'b1);

        rd_en = 1'b1;
        for (int a = 0; a < TILE; a++) begin
            step(1'b0);
            check_val($sformatf("pop %0d (FIFO order)", a), rd_data, pat(a));
        end
        step(1'b0);
        check_bit("empty after the last pop", empty, 1'b1);
        // one more read on the empty FIFO: rd_data holds, nothing pops
        scratch = rd_data;
        step(1'b0);
        check_val("rd_data holds on empty FIFO", rd_data, scratch);
        check_bit("still empty after the extra read", empty, 1'b1);
        rd_en = 1'b0;

        // ---- [3] Write while full is dropped --------------------------------
        $display("[3] Write while full is ignored (even with a read in the same clock)");
        for (int a = 0; a < TILE; a++) begin
            wr_en = 1'b1; wr_data = pat(10 + a);
            step(1'b0);
        end
        check_bit("full before the dropped push", full, 1'b1);
        wr_en = 1'b1; wr_data = pat(99);           // must be dropped
        step(1'b0);
        check_bit("full still set after the dropped push", full, 1'b1);
        wr_en = 1'b0;

        // read + write while full: only the read takes effect
        wr_en = 1'b1; wr_data = pat(98);
        rd_en = 1'b1;
        step(1'b0);
        check_val("pop head while full (write dropped)", rd_data, pat(10));
        wr_en = 1'b0;
        for (int a = 1; a < TILE; a++) begin
            step(1'b0);
            check_val($sformatf("pop %0d after dropped writes", a), rd_data, pat(10 + a));
        end
        step(1'b0);
        check_bit("empty after the drained queue", empty, 1'b1);
        check_val("pat(98)/pat(99) never entered the FIFO", rd_data, pat(TILE-1+10));
        rd_en = 1'b0;

        // ---- [4] Read + write in the same clock ------------------------------
        // both must take effect: head pops, new element goes to the tail
        $display("[4] Simultaneous rd_en + wr_en: both take effect, order kept");
        wr_en = 1'b1; wr_data = pat(20);
        step(1'b0);
        wr_data = pat(21);
        step(1'b0);
        wr_en = 1'b0;
        check_bit("two entries present", empty, 1'b0);

        rd_en = 1'b1; wr_en = 1'b1; wr_data = pat(22);   // the tested clock
        step(1'b0);
        check_val("rd_data is the old head on the same clock", rd_data, pat(20));
        check_bit("length unchanged by simultaneous rd+wr", empty, 1'b0);
        wr_en = 1'b0;
        step(1'b0);
        check_val("next pop is the 2nd entry", rd_data, pat(21));
        step(1'b0);
        check_val("3rd pop is the entry written during the rd+wr clock",
                  rd_data, pat(22));
        step(1'b0);
        check_bit("empty after draining the 3 pops", empty, 1'b1);
        rd_en = 1'b0;

        // ---- [5] Read + write while empty ------------------------------------
        $display("[5] Simultaneous rd_en + wr_en on an empty FIFO: only the push");
        rd_en = 1'b1; wr_en = 1'b1; wr_data = pat(30);
        step(1'b0);
        check_bit("not empty after the push", empty, 1'b0);
        wr_en = 1'b0;
        step(1'b0);
        check_val("pop of the pushed entry", rd_data, pat(30));
        step(1'b0);
        check_bit("empty after popping it", empty, 1'b1);
        rd_en = 1'b0;

        // ---- [6] Randomized stream -------------------------------------------
        $display("[6] Randomized push/pop stream (%0d cycles) vs the mirror model",
                 RUN_CYCLES);
        errs0 = errors;
        for (int c = 0; c < RUN_CYCLES; c++) begin
            if (c % 97 == 0) begin                 // periodic reset
                wr_en = $urandom_range(0, 1);
                rd_en = $urandom_range(0, 1);
                for (int w = 0; w < DEPTH; w++)
                    wr_data[w*WIDTH +: WIDTH] = WIDTH'($urandom);
                step(1'b1);
            end else begin
                wr_en = ($urandom_range(0, 2) != 0);   // ~67% active
                rd_en = ($urandom_range(0, 2) != 0);
                for (int w = 0; w < DEPTH; w++)
                    wr_data[w*WIDTH +: WIDTH] = WIDTH'($urandom);
                step(1'b0);
            end
        end
        wr_en = 1'b0;

        // drain whatever is left and compare against the mirror
        errs0 = errors;
        rd_en = 1'b1;
        while (msize > 0) begin
            scratch = model[0];
            step(1'b0);
            check_val("drain entry", rd_data, scratch);
        end
        step(1'b0);
        check_bit("empty after the drain", empty, 1'b1);
        rd_en = 1'b0;
        step(1'b0);
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-46s", "random stream: no failures");

        // ---- [7] Single-entry instance (TILE=1) ------------------------------
        $display("[7] Single-entry FIFO (TILE=1): full/empty corner");
        step(1'b1);
        check_bit("T1: empty after reset", empty1, 1'b1);
        check_bit("T1: not full after reset", full1, 1'b0);
        wr_en1 = 1'b1; wr_data1 = 8'h5A;
        step(1'b0);
        check_bit("T1: full after one push", full1, 1'b1);
        check_bit("T1: not empty after one push", empty1, 1'b0);
        wr_en1 = 1'b1; wr_data1 = 8'hA5;           // dropped: full
        step(1'b0);
        check_bit("T1: still full after the dropped push", full1, 1'b1);
        wr_en1 = 1'b0;
        rd_en1 = 1'b1;
        step(1'b0);
        check_val1("T1: pop returns the first entry", rd_data1, 8'h5A);
        step(1'b0);
        check_bit("T1: empty after the pop", empty1, 1'b1);
        rd_en1 = 1'b0;

        $fclose(fd);

        // ---- Summary ---------------------------------------------------------
        $display("=============================================================");
        $display(" WeightFIFO tb summary: %0d mirror-checked cycles, %0d checks, %0d failures",
                 cycles_checked, checks, errors);
        if (errors == 0)
            $display(" *** WEIGHT FIFO TB PASSED ***");
        else
            $display(" *** WEIGHT FIFO TB FAILED ***");
        $display("=============================================================");
        if (errors != 0) $fatal(1, "WeightFIFO tb failed");
        $finish;
    end

endmodule

`default_nettype wire
