`timescale 1ns / 1ps
// Self-checking testbench for SPI_driver
// Prints PASS/FAIL for every check and a summary at the end.

module SPI_driver_tb;

    // Waveform file for GTKWave
    initial begin
        $dumpfile("spi_driver.vcd");
        $dumpvars(0, SPI_driver_tb);
    end

    // ------------------------------------------------------------------
    // DUT signals
    // ------------------------------------------------------------------
    reg        clk = 0;
    reg        rst = 1;
    reg        fast = 1;
    reg        start = 0;
    reg  [7:0] tx_data = 8'h00;
    wire [7:0] rx_data;
    wire       done;
    wire       busy;
    wire       miso;
    wire       mosi;
    wire       sclk;

    SPI_driver dut (
        .clk(clk), .rst(rst), .fast(fast), .fast_half(9'd2),
        .start(start), .tx_data(tx_data),
        .rx_data(rx_data), .done(done), .busy(busy),
        .miso(miso), .mosi(mosi), .sclk(sclk)
    );

    always #5 clk = ~clk;   // 100 MHz

    // ------------------------------------------------------------------
    // Fake SD card (SPI mode 0 slave)
    //   - reads MOSI on rising SCLK
    //   - changes MISO on falling SCLK
    // loopback = 1 ties MISO straight to MOSI instead
    // ------------------------------------------------------------------
    reg        loopback = 1;
    reg  [7:0] slave_tx = 8'hFF;   // byte the fake card sends
    reg  [7:0] slave_rx = 8'h00;   // byte the fake card received

    assign miso = loopback ? mosi : slave_tx[7];

    always @(posedge sclk) slave_rx <= {slave_rx[6:0], mosi};
    always @(negedge sclk) slave_tx <= {slave_tx[6:0], 1'b1};

    // ------------------------------------------------------------------
    // Monitors: count SCLK rising edges, measure SCLK period, done width
    // ------------------------------------------------------------------
    integer sclk_edges = 0;
    realtime last_rise = 0, sclk_period = 0;
    integer done_width = 0, done_width_max = 0;

    always @(posedge sclk) begin
        sclk_edges = sclk_edges + 1;
        if (last_rise != 0) sclk_period = $realtime - last_rise;
        last_rise = $realtime;
    end

    always @(posedge clk) begin
        if (done) done_width = done_width + 1;
        else begin
            if (done_width > done_width_max) done_width_max = done_width;
            done_width = 0;
        end
    end

    // ------------------------------------------------------------------
    // Bookkeeping
    // ------------------------------------------------------------------
    integer pass_count = 0, fail_count = 0;

    task check(input cond, input [8*64-1:0] msg);
        begin
            if (cond) begin
                pass_count = pass_count + 1;
                $display("  PASS: %0s", msg);
            end else begin
                fail_count = fail_count + 1;
                $display("  FAIL: %0s   <-----", msg);
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Send one byte and wait for done (with timeout)
    // ------------------------------------------------------------------
    reg timed_out;
    reg busy_seen;
    integer cycles;

    task send_byte(input [7:0] b);
        begin
            timed_out  = 0;
            busy_seen  = 0;
            sclk_edges = 0;
            last_rise  = 0;
            done_width_max = 0;

            @(negedge clk);
            tx_data = b;
            start   = 1;
            @(negedge clk);
            start   = 0;

            cycles = 0;
            while (!done && cycles < 20000) begin
                if (busy) busy_seen = 1;
                @(negedge clk);
                cycles = cycles + 1;
            end
            if (!done) timed_out = 1;
            @(negedge clk);   // let done drop so width gets recorded
            @(negedge clk);
        end
    endtask

    // ------------------------------------------------------------------
    // Tests
    // ------------------------------------------------------------------
    reg [7:0] patterns [0:5];
    integer i;

    initial begin
        patterns[0] = 8'hA5;
        patterns[1] = 8'h00;
        patterns[2] = 8'hFF;
        patterns[3] = 8'h3C;
        patterns[4] = 8'h81;
        patterns[5] = 8'h40;   // CMD0 first byte

        $display("");
        $display("==============================================");
        $display(" SPI_driver testbench");
        $display("==============================================");

        // ---------------- Reset ----------------
        repeat (5) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        $display("");
        $display("[TEST 1] Idle state after reset");
        check(sclk == 1'b0, "SCLK idles low");
        check(mosi == 1'b1, "MOSI idles high");
        check(busy == 1'b0, "busy is low");
        check(done == 1'b0, "done is low");

        // ---------------- Loopback, fast ----------------
        $display("");
        $display("[TEST 2] Loopback (MISO tied to MOSI), fast clock");
        loopback = 1;
        fast     = 1;
        for (i = 0; i < 6; i = i + 1) begin
            send_byte(patterns[i]);
            $display("  sent 0x%02h  received 0x%02h  sclk edges=%0d",
                     patterns[i], rx_data, sclk_edges);
            check(!timed_out,              "transfer finished (no timeout)");
            check(rx_data == patterns[i],  "received byte matches sent byte");
            check(sclk_edges == 8,         "exactly 8 SCLK pulses");
            check(busy_seen,               "busy went high during transfer");
            check(done_width_max == 1,     "done pulse is 1 clock wide");
            check(sclk == 0 && mosi == 1,  "SCLK low and MOSI high afterwards");
        end

        // ---------------- Fake card, both directions ----------------
        $display("");
        $display("[TEST 3] Fake SD card: separate send and receive bytes");
        loopback = 0;
        fast     = 1;
        begin : fake_card_tests
            reg [7:0] host_b [0:3];
            reg [7:0] card_b [0:3];
            host_b[0] = 8'h40; card_b[0] = 8'h01;   // CMD0 -> R1 idle
            host_b[1] = 8'hFF; card_b[1] = 8'hFE;   // filler -> data token
            host_b[2] = 8'h48; card_b[2] = 8'hAA;   // CMD8 -> echo
            host_b[3] = 8'h95; card_b[3] = 8'h5A;
            for (i = 0; i < 4; i = i + 1) begin
                slave_tx = card_b[i];   // card has first bit ready before first edge
                slave_rx = 8'h00;
                send_byte(host_b[i]);
                $display("  FPGA sent 0x%02h (card got 0x%02h) | card sent 0x%02h (FPGA got 0x%02h)",
                         host_b[i], slave_rx, card_b[i], rx_data);
                check(slave_rx == host_b[i], "card received correct byte (MOSI timing OK)");
                check(rx_data  == card_b[i], "FPGA received correct byte (MISO timing OK)");
            end
        end

        // ---------------- Clock speeds ----------------
        $display("");
        $display("[TEST 4] SCLK frequency");
        loopback = 1;

        fast = 1;
        send_byte(8'h55);
        $display("  fast mode SCLK period = %0.1f ns (%0.2f MHz)",
                 sclk_period, 1000.0 / sclk_period);
        check(sclk_period > 39.0 && sclk_period < 41.0, "fast mode = 25 MHz (40 ns)");

        fast = 0;
        send_byte(8'h55);
        $display("  slow mode SCLK period = %0.1f ns (%0.1f kHz)",
                 sclk_period, 1000000.0 / sclk_period);
        check(sclk_period > 4999.0 && sclk_period < 5001.0, "slow mode = 200 kHz (5000 ns)");
        check(rx_data == 8'h55, "slow mode loopback byte correct");

        // ---------------- Back-to-back ----------------
        $display("");
        $display("[TEST 5] Back-to-back bytes (like the 10 wake-up bytes)");
        fast = 1;
        for (i = 0; i < 10; i = i + 1) begin
            send_byte(8'hFF);
            check(rx_data == 8'hFF && sclk_edges == 8, "0xFF byte sent and received");
        end

        // ---------------- Summary ----------------
        $display("");
        $display("==============================================");
        $display(" RESULTS: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display(" ALL TESTS PASSED");
        else                 $display(" SOME TESTS FAILED - see lines marked <-----");
        $display("==============================================");
        $display("");
        $finish;
    end

    // Global safety timeout
    initial begin
        #10_000_000;
        $display("GLOBAL TIMEOUT - simulation hung");
        $finish;
    end

endmodule