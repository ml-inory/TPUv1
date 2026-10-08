//=============================================================================
// UB (unified buffer) testbench
// DUT: rtl/ub.sv - single-port synchronous memory, 1-cycle registered read,
//                  synchronous reset that clears the whole array.
//
// Self-checking: the tb keeps a mirror copy of the memory and compares
// dut.rd_data against it on every clock (once a read has made it defined),
// on top of the directed checks below. tb/ub/ub_tb.py repeats the data check
// independently from the dumped trace.
//
// Directed checks:
//   [1] reset clears the memory and rd_data, and wins over a concurrent write
//   [2] write/read walk of every address, back-to-back reads at 1/cycle
//   [3] rd_en=0 holds rd_data; wr_en=0 leaves the memory untouched
//   [4] simultaneous read+write of the same address returns the OLD value
//   [5] randomized rd/wr stream (with resets) vs the mirror model
//
//   make ub_tb        -> run vvp, then the Python reference check
//   +trace=<file>     trace path  (default: ub_trace.txt)
//   +wave=<file>      VCD path    (default: ub_tb.vcd)
//
// Trace format (one line per clock; '#' header carries the parameters):
//   cyc rst rd_en rd_addr wr_en wr_addr wr_data rd_data
// The very first cycle is a reset, so rd_data is defined from the first
// dumped clock on and every cycle can be compared.
//=============================================================================
`default_nettype none
`timescale 1ns/1ps

module ub_tb;

    localparam int WIDTH      = 8;
    localparam int DEPTH      = 64;    // power of 2: no out-of-range addresses
    localparam int AW         = $clog2(DEPTH);
    localparam int CLK_PERIOD = 10;    // ns
    localparam int RUN_CYCLES = 600;

    // ---- DUT signals --------------------------------------------------------
    logic clk, rst;

    logic             rd_en;
    logic [AW-1:0]    rd_addr;
    logic [WIDTH-1:0] rd_data;

    logic             wr_en;
    logic [AW-1:0]    wr_addr;
    logic [WIDTH-1:0] wr_data;

    // ---- DUT ----------------------------------------------------------------
    UB #(
        .WIDTH(WIDTH), .DEPTH(DEPTH)
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

    logic [WIDTH-1:0] model [0:DEPTH-1];   // mirror of dut.mem
    logic [WIDTH-1:0] rd_exp;              // expected rd_data (0 after reset)

    function automatic logic [WIDTH-1:0] pat(input int i);
        pat = i * 37 + 11;                 // truncated to WIDTH bits
    endfunction

    task automatic check_bit(input string name, input logic got, input logic exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-38s got=%b  exp=%b", name, got, exp);
        end else begin
            $display("  [PASS] %-38s = %b", name, got);
        end
    endtask

    task automatic check_val(input string name, input logic [WIDTH-1:0] got,
                             input logic [WIDTH-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-38s got=%02h  exp=%02h", name, got, exp);
        end else begin
            $display("  [PASS] %-38s = %02h", name, got);
        end
    endtask

    // data check inside big loops: report failures only (keeps the log short)
    task automatic check_val_q(input string name, input logic [WIDTH-1:0] got,
                               input logic [WIDTH-1:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [FAIL] %-38s got=%02h  exp=%02h", name, got, exp);
        end
    endtask

    // ---- Trace dump ---------------------------------------------------------
    task automatic dump();
        $fwrite(fd, "%0d %0d %0d %0d %0d %0d %0d %0d\n",
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
                $display("  [FAIL] cyc %0d rd_data got=%02h exp=%02h (rst=%b rd_en=%b rd_addr=%0d wr_en=%b wr_addr=%0d wr_data=%02h)",
                         cyc, rd_data, rd_exp, rst, rd_en, rd_addr,
                         wr_en, wr_addr, wr_data);
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
        $fwrite(fd, "# UB WIDTH=%0d DEPTH=%0d\n", WIDTH, DEPTH);

        // input defaults
        rst = 1'b0;
        rd_en = 1'b0; rd_addr = '0;
        wr_en = 1'b0; wr_addr = '0; wr_data = '0;

        $display("=================================================");
        $display(" UB testbench  (WIDTH=%0d, DEPTH=%0d, AW=%0d)", WIDTH, DEPTH, AW);
        $display(" trace -> %s   (reference model: ub_tb.py)", trace_file);
        $display("=================================================");

        // ---- [1] Reset ------------------------------------------------------
        $display("[1] Reset clears the memory and rd_data, wins over a write");
        // every cycle below is compared against the mirror from the start
        step(1'b1);                                  // rd_en=0: isolates rd_data
        check_bit("rd_data defined after reset", !$isunknown(rd_data), 1'b1);
        check_val("rd_data cleared by reset", rd_data, 8'h00);

        wr_en = 1'b1; wr_addr = 5; wr_data = 8'hA5;
        step(1'b0);
        wr_addr = 9; wr_data = 8'h3C;
        step(1'b0);
        wr_en = 1'b0;

        rd_en = 1'b1; rd_addr = 5;
        step(1'b0);
        check_val("mem[5] before reset", rd_data, 8'hA5);
        rd_addr = 9;
        step(1'b0);
        check_val("mem[9] before reset", rd_data, 8'h3C);

        // one reset cycle with a write request in flight: it must be dropped
        rd_en = 1'b0;
        wr_en = 1'b1; wr_addr = 5; wr_data = 8'h77;
        step(1'b1);
        wr_en = 1'b0;
        check_val("rd_data cleared by reset (write raced)", rd_data, 8'h00);

        rd_en = 1'b1; rd_addr = 5;
        step(1'b0);
        check_val("mem[5] after reset", rd_data, 8'h00);
        rd_addr = 9;
        step(1'b0);
        check_val("mem[9] after reset", rd_data, 8'h00);

        // whole array must be zero after a reset
        $display("       full memory image after reset");
        errs0 = errors;
        for (int i = 0; i < DEPTH; i++) begin
            rd_addr = i[AW-1:0];
            step(1'b0);
            check_val_q($sformatf("post-reset mem[%0d]", i), rd_data, '0);
        end
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-38s (%0d addresses)", "post-reset memory all zero",
                     DEPTH);
        rd_en = 1'b0;

        // ---- [2] Full write walk + back-to-back read walk --------------------
        $display("[2] Write all %0d addresses, then read them back at 1/cycle",
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
            check_val_q($sformatf("mem[%0d] read-back", i), rd_data, pat(i));
        end
        rd_en = 1'b0;
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-38s (%0d addresses)", "write/read walk", DEPTH);

        // ---- [3] Read hold / write disable -----------------------------------
        $display("[3] rd_en=0 holds rd_data; wr_en=0 leaves memory untouched");
        rd_en = 1'b1; rd_addr = 7;
        step(1'b0);
        check_val("mem[7] (reference read)", rd_data, pat(7));
        rd_en = 1'b0; rd_addr = 20;            // address changes, no read enable
        step(1'b0);
        check_val("rd_data held while rd_en=0", rd_data, pat(7));
        wr_en = 1'b0; wr_addr = 7; wr_data = 8'h00;   // masked write request
        step(1'b0);
        check_val("rd_data held while wr_en=0", rd_data, pat(7));
        rd_en = 1'b1; rd_addr = 7;
        step(1'b0);
        check_val("mem[7] unchanged by wr_en=0", rd_data, pat(7));
        rd_en = 1'b0;

        // ---- [4] Same-cycle read + write -------------------------------------
        $display("[4] Simultaneous read+write: the read sees the OLD value");
        rd_en = 1'b1; rd_addr = 12;
        step(1'b0);
        check_val("mem[12] (pre-write)", rd_data, pat(12));

        rd_addr = 12; wr_en = 1'b1; wr_addr = 12; wr_data = 8'h5A;
        step(1'b0);
        check_val("rd_data (same addr, same cycle -> old)", rd_data, pat(12));

        wr_en = 1'b0;                          // rd_en still 1: new value next cycle
        step(1'b0);
        check_val("rd_data (new value one cycle later)", rd_data, 8'h5A);

        wr_en = 1'b1; wr_addr = 20; wr_data = 8'h3C;   // write to another address
        step(1'b0);
        check_val("rd_data (write to other addr)", rd_data, 8'h5A);
        wr_en = 1'b0;

        rd_addr = 20;
        step(1'b0);
        check_val("mem[20] (write to other addr)", rd_data, 8'h3C);
        rd_en = 1'b0;

        // ---- [5] Randomized stream -------------------------------------------
        $display("[5] Randomized stream (%0d cycles) vs the mirror model",
                 RUN_CYCLES);
        for (int c = 0; c < RUN_CYCLES; c++) begin
            if (c % 97 == 0) begin               // occasional reset, raced with a write
                rd_en   = $urandom_range(0, 1);
                rd_addr = AW'($urandom_range(0, DEPTH-1));
                wr_en   = $urandom_range(0, 1);
                wr_addr = AW'($urandom_range(0, DEPTH-1));
                wr_data = WIDTH'($urandom);
                step(1'b1);
            end else begin
                rd_en   = ($urandom_range(0, 3) != 0);   // ~75% active
                rd_addr = AW'($urandom_range(0, DEPTH-1));
                wr_en   = ($urandom_range(0, 3) != 0);
                wr_addr = AW'($urandom_range(0, DEPTH-1));
                wr_data = WIDTH'($urandom);
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
            check_val_q($sformatf("post-stress mem[%0d]", i), rd_data, model[i]);
        end
        rd_en = 1'b0;
        checks++;
        if (errors == errs0)
            $display("  [PASS] %-38s (%0d addresses)", "post-stress sweep vs mirror",
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
