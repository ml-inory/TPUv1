//=============================================================================
// UB (unified buffer) testbench
// DUT: rtl/ub.sv - line-addressed synchronous memory
//
// One address holds a whole line of WIDTH*WORD_SIZE bits: WORD_SIZE words of
// WIDTH bits each (the MXU act path keeps one act vector per line). The read
// is registered (1 cycle of latency), the write is synchronous, and reset
// clears the whole array AND rd_data.
//
// Directed checks:
//   [1] reset clears rd_data (with rd_en=0) and the whole memory image
//   [2] write/read walk of every line, back-to-back reads at 1 line/cycle
//   [3] rd_en=0 holds rd_data; wr_en=0 leaves the memory untouched
//   [4] simultaneous read+write of the same line returns the OLD line
//   [5] single-word lanes: a write only lands in the addressed line/lane
//   [6] randomized line stream (with resets) vs the mirror model
//
// tb/ub/ub_tb.py repeats the data check independently from the dumped trace.
//
//   make ub_tb        -> run vvp, then the Python reference check
//   +trace=<file>     trace path (default ub_trace.txt)
//   +wave=<file>      VCD path   (default ub_tb.vcd)
//
// Trace format (one line per clock; '#' header carries the parameters):
//   cyc rst rd_en rd_addr wr_en wr_addr wr_data rd_data        (hex)
// The very first cycle is a reset, so rd_data is defined from the first
// dumped clock on and every cycle can be compared.
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module ub_tb;

    localparam int WIDTH      = 8;               // bits per word
    localparam int WORD_SIZE  = 8;               // words per line (lane count)
    localparam int LINE       = WIDTH*WORD_SIZE; // bits per line
    localparam int DEPTH      = 64;              // lines
    localparam int AW         = $clog2(DEPTH);
    localparam int CLK_PERIOD = 10;              // ns
    localparam int RUN_CYCLES = 300;

    // ---- DUT signals --------------------------------------------------------
    logic clk, rst;

    logic             rd_en;
    logic [AW-1:0]    rd_addr;
    logic [LINE-1:0]  rd_data;

    logic             wr_en;
    logic [AW-1:0]    wr_addr;
    logic [LINE-1:0]  wr_data;

    // ---- DUT ----------------------------------------------------------------
    UB #(
        .WIDTH(WIDTH), .WORD_SIZE(WORD_SIZE), .DEPTH(DEPTH)
    ) dut (
        .clk     (clk),
        .rst     (rst),
        .rd_en   (rd_en),
        .rd_addr (rd_addr),
        .rd_data (rd_data),
        .wr_en   (wr_en),
        .wr_addr (wr_addr),
        .wr_data (wr_data)
    );

    // ---- Clock --------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- Bookkeeping / mirror model ----------------------------------------
    int errors         = 0;     // directed + mirror failures
    int checks         = 0;     // directed checks (mirror cycles counted apart)
    int cycles_checked = 0;     // clocks the mirror comparison ran on
    int errs0;                  // snapshot used by loop summaries
    int cyc    = 0;
    integer fd;

    logic [LINE-1:0] model  [0:DEPTH-1];   // mirror of dut.mem
    logic [LINE-1:0] rd_exp;               // expected rd_data (0 after reset)
    logic [LINE-1:0] exp_tmp;              // scratch for lane-by-lane checks

    // line pattern: every lane gets its own value, so a lane swap shows up
    function automatic logic [LINE-1:0] pat(input int a);
        logic [LINE-1:0] p;
        p = '0;
        for (int w = 0; w < WORD_SIZE; w++)
            p[w*WIDTH +: WIDTH] = WIDTH'((a*WORD_SIZE + w) * 37 + 11);
        pat = p;
    endfunction

    // index of the first lane that differs (-1 when the lines match)
    function automatic int bad_lane(input logic [LINE-1:0] got,
                                    input logic [LINE-1:0] exp);
        bad_lane = -1;
        for (int w = 0; w < WORD_SIZE; w++)
            if (bad_lane < 0 && got[w*WIDTH +: WIDTH] !== exp[w*WIDTH +: WIDTH])
                bad_lane = w;
    endfunction

    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-42s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-42s = %b", name, got);
        end
    endtask

    task automatic check_word(input string name, input logic [WIDTH-1:0] got,
                              input logic [WIDTH-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-42s got=%02h  exp=%02h", name, got, exp);
        end else begin
            $display("  [PASS] %-42s = %02h", name, got);
        end
    endtask

    task automatic check_line(input string name, input logic [LINE-1:0] got,
                              input logic [LINE-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-42s got=%h exp=%h (first bad lane %0d)",
                     name, got, exp, bad_lane(got, exp));
        end else begin
            $display("  [PASS] %-42s = %h", name, got);
        end
    endtask

    // data check inside big loops: report failures only (keeps the log short)
    task automatic check_line_q(input string name, input logic [LINE-1:0] got,
                                input logic [LINE-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-42s got=%h exp=%h (first bad lane %0d)",
                     name, got, exp, bad_lane(got, exp));
        end
    endtask

    // ---- Trace dump ---------------------------------------------------------
    task automatic dump();
        $fwrite(fd, "%0d %0d %0d %0d %0d %0d %h %h\n",
                cyc, rst, rd_en, rd_addr, wr_en, wr_addr, wr_data, rd_data);
    endtask

    // drive rst on the falling edge, advance one cycle, then dump
    task automatic step(input logic r);
        @(negedge clk);
        rst = r;
        // one clock of the mirror, exactly like rtl/ub.sv (reset wins, the
        // read samples the OLD memory contents, i.e. before this cycle's write)
        if (r) begin
            for (int i = 0; i < DEPTH; i++) model[i] = '0;
            rd_exp = '0;
        end else begin
            if (rd_en) rd_exp = model[rd_addr];
            if (wr_en) model[wr_addr] = wr_data;
        end
        @(posedge clk); #1;
        cyc = cyc + 1;
        dump();
        cycles_checked++;
        if (rd_data !== rd_exp) begin
            errors++;
            if (errors <= 20)
                $display("  [FAIL] cyc %0d rd_data got=%h exp=%h (first bad lane %0d) (rst=%b rd_en=%b rd_addr=%0d wr_en=%b wr_addr=%0d wr_data=%h)",
                         cyc, rd_data, rd_exp, bad_lane(rd_data, rd_exp), rst,
                         rd_en, rd_addr, wr_en, wr_addr, wr_data);
        end
    endtask

    // ---- Waveform dump / watchdog -------------------------------------------
    initial begin
        string wave_file;
        if (!$value$plusargs("wave=%s", wave_file))
            wave_file = "ub_tb.vcd";
        $dumpfile(wave_file);
        $dumpvars(0, ub_tb);
    end

    initial begin
        #(CLK_PERIOD * 100000);
        $fatal(1, "TIMEOUT: simulation did not finish");
    end

    // ---- Stimulus -----------------------------------------------------------
    initial begin
        string trace_file;
        if (!$value$plusargs("trace=%s", trace_file))
            trace_file = "ub_trace.txt";
        fd = $fopen(trace_file, "w");
        if (fd == 0)
            $fatal(1, "cannot open trace file '%s'", trace_file);
        $fwrite(fd, "# UB WIDTH=%0d WORD_SIZE=%0d DEPTH=%0d\n",
                WIDTH, WORD_SIZE, DEPTH);

        // input defaults
        rst = 1'b0;
        rd_en = 1'b0; rd_addr = '0;
        wr_en = 1'b0; wr_addr = '0; wr_data = '0;

        $display("=================================================");
        $display(" UB testbench  (WIDTH=%0d bits/word, WORD_SIZE=%0d words/line,",
                 WIDTH, WORD_SIZE);
        $display("                LINE=%0d bits, DEPTH=%0d lines, AW=%0d)",
                 LINE, DEPTH, AW);
        $display(" trace -> %s   (reference model: ub_tb.py)", trace_file);
        $display("=================================================");

        // ---- [1] Reset ------------------------------------------------------
        $display("[1] Reset clears the memory image and rd_data");
        step(1'b1);                                  // rd_en=0: isolates rd_data
        check_bit("rd_data defined after reset", !$isunknown(rd_data), 1'b1);
        check_line("rd_data cleared by reset", rd_data, '0);

        wr_en = 1'b1; wr_addr = 5; wr_data = pat(5);
        step(1'b0);
        wr_addr = 9; wr_data = pat(9);
        step(1'b0);
        wr_en = 1'b0;

        rd_en = 1'b1; rd_addr = 5;
        step(1'b0);
        check_line("line[5] before reset", rd_data, pat(5));
        rd_addr = 9;
        step(1'b0);
        check_line("line[9] before reset", rd_data, pat(9));

        // one reset cycle with a write request in flight: it must be dropped
        rd_en = 1'b0;
        wr_en = 1'b1; wr_addr = 5; wr_data = pat(11);
        step(1'b1);
        wr_en = 1'b0;
        check_line("rd_data cleared by reset (write raced)", rd_data, '0);

        rd_en = 1'b1; rd_addr = 5;
        step(1'b0);
        check_line("line[5] after reset", rd_data, '0);
        rd_addr = 9;
        step(1'b0);
        check_line("line[9] after reset", rd_data, '0);

        // whole array must be zero after a reset
        $display("       full memory image after reset");
        errs0 = errors;
        for (int i = 0; i < DEPTH; i++) begin
            rd_addr = i[AW-1:0];
            step(1'b0);
            check_line_q($sformatf("post-reset line[%0d]", i), rd_data, '0);
        end
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-42s (%0d lines)", "post-reset memory all zero",
                     DEPTH);
        rd_en = 1'b0;

        // ---- [2] Full write walk + back-to-back read walk --------------------
        $display("[2] Write all %0d lines, then read them back at 1 line/cycle",
                 DEPTH);
        errs0 = errors;
        wr_en = 1'b1;
        for (int i = 0; i < DEPTH; i++) begin
            wr_addr = i[AW-1:0];
            wr_data = pat(i);
            step(1'b0);
        end
        wr_en = 1'b0;
        rd_en = 1'b1;
        for (int i = 0; i < DEPTH; i++) begin
            rd_addr = i[AW-1:0];       // address advances every cycle
            step(1'b0);
            check_line_q($sformatf("line[%0d] read-back", i), rd_data, pat(i));
        end
        rd_en = 1'b0;
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-42s (%0d lines)", "write/read walk", DEPTH);

        // ---- [3] Read hold / write disable -----------------------------------
        $display("[3] rd_en=0 holds rd_data; wr_en=0 leaves memory untouched");
        rd_en = 1'b1; rd_addr = 7;
        step(1'b0);
        check_line("line[7] (reference read)", rd_data, pat(7));
        rd_en = 1'b0; rd_addr = 20;               // address changes, no read enable
        step(1'b0);
        check_line("rd_data held while rd_en=0", rd_data, pat(7));
        wr_en = 1'b0; wr_addr = 7; wr_data = '0;  // masked write request
        step(1'b0);
        check_line("rd_data held while wr_en=0", rd_data, pat(7));
        rd_en = 1'b1; rd_addr = 7;
        step(1'b0);
        check_line("line[7] unchanged by wr_en=0", rd_data, pat(7));
        rd_en = 1'b0;

        // ---- [4] Same-cycle read + write -------------------------------------
        $display("[4] Simultaneous read+write: the read sees the OLD line");
        rd_en = 1'b1; rd_addr = 12;
        step(1'b0);
        check_line("line[12] (pre-write)", rd_data, pat(12));

        rd_addr = 12; wr_en = 1'b1; wr_addr = 12; wr_data = pat(33);
        step(1'b0);
        check_line("rd_data (same line, same cycle -> old)", rd_data, pat(12));

        wr_en = 1'b0;                        // rd_en still 1: new line next cycle
        step(1'b0);
        check_line("rd_data (new line one cycle later)", rd_data, pat(33));

        wr_en = 1'b1; wr_addr = 20; wr_data = pat(44);   // write to another line
        step(1'b0);
        check_line("rd_data (write to other line)", rd_data, pat(33));
        wr_en = 1'b0;

        rd_addr = 20;
        step(1'b0);
        check_line("line[20] (write to other line)", rd_data, pat(44));
        rd_en = 1'b0;

        // ---- [5] Word lanes ---------------------------------------------------
        // a line with only one lane set must come back with exactly that lane
        $display("[5] Word lanes: only the addressed lane carries a value");
        for (int w = 0; w < WORD_SIZE; w += WORD_SIZE-1) begin
            wr_data = '0;
            wr_data[w*WIDTH +: WIDTH] = 8'hAA;
            wr_en = 1'b1; wr_addr = 30 + w;
            step(1'b0);
            wr_en = 1'b0;
            rd_en = 1'b1; rd_addr = 30 + w;
            step(1'b0);
            check_line($sformatf("line[%0d] single lane %0d", 30+w, w),
                       rd_data, wr_data);
            rd_en = 1'b0;
        end
        // and every lane of a line keeps its own word
        rd_en = 1'b1; rd_addr = 3;
        step(1'b0);
        exp_tmp = pat(3);
        for (int w = 0; w < WORD_SIZE; w++)
            check_word($sformatf("line[3] lane %0d", w),
                       rd_data[w*WIDTH +: WIDTH], exp_tmp[w*WIDTH +: WIDTH]);
        rd_en = 1'b0;

        // ---- [6] Randomized stream -------------------------------------------
        $display("[6] Randomized line stream (%0d cycles) vs the mirror model",
                 RUN_CYCLES);
        for (int c = 0; c < RUN_CYCLES; c++) begin
            if (c % 97 == 0) begin           // occasional reset, raced with a write
                rd_en   = $urandom_range(0, 1);
                rd_addr = AW'($urandom_range(0, DEPTH-1));
                wr_en   = $urandom_range(0, 1);
                wr_addr = AW'($urandom_range(0, DEPTH-1));
                for (int w = 0; w < WORD_SIZE; w++)
                    wr_data[w*WIDTH +: WIDTH] = WIDTH'($urandom);
                step(1'b1);
            end else begin
                rd_en   = ($urandom_range(0, 3) != 0);   // ~75% active
                rd_addr = AW'($urandom_range(0, DEPTH-1));
                wr_en   = ($urandom_range(0, 3) != 0);
                wr_addr = AW'($urandom_range(0, DEPTH-1));
                for (int w = 0; w < WORD_SIZE; w++)
                    wr_data[w*WIDTH +: WIDTH] = WIDTH'($urandom);
                step(1'b0);
            end
        end

        // settle, then read the whole image back and compare with the mirror
        rd_en = 1'b0; wr_en = 1'b0;
        step(1'b0);
        $display("       post-stress read-back sweep");
        errs0 = errors;
        rd_en = 1'b1;
        for (int i = 0; i < DEPTH; i++) begin
            rd_addr = i[AW-1:0];
            step(1'b0);
            check_line_q($sformatf("post-stress line[%0d]", i), rd_data, model[i]);
        end
        rd_en = 1'b0;
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-42s (%0d lines)", "post-stress sweep vs mirror",
                     DEPTH);

        $fclose(fd);

        // ---- Summary ---------------------------------------------------------
        $display("=================================================");
        $display(" UB tb summary: %0d mirror-checked cycles, %0d directed checks, %0d failures",
                 cycles_checked, checks, errors);
        if (errors == 0)
            $display(" *** UB TB PASSED *** (data check repeated by ub_tb.py)");
        else
            $display(" *** UB TB FAILED ***");
        $display("=================================================");
        if (errors != 0) $fatal(1, "UB tb failed");
        $finish;
    end

endmodule

`default_nettype wire
