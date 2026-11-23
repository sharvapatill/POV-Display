`timescale 1ns / 1ps
// Self-checking testbench for SD_cmd + SPI_driver
// A fake SD card records the 6 command bytes it receives, then answers
// after a chosen number of 0xFF "thinking" bytes.

module SD_cmd_tb;

    // Waveform file for GTKWave
    initial begin
        $dumpfile("sd_cmd.vcd");
        $dumpvars(0, SD_cmd_tb);
    end

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;   // 100 MHz

    // ---------------- Command engine <-> controller ----------------
    reg         cmd_start = 0;
    reg  [5:0]  cmd_index = 0;
    reg  [31:0] cmd_arg   = 0;
    reg  [2:0]  extra_bytes = 0;
    wire        cmd_done, cmd_busy, timeout;
    wire [7:0]  r1;
    wire [31:0] resp_data;

    // ---------------- Command engine <-> SPI ----------------
    wire       spi_start, spi_done, spi_busy;
    wire [7:0] spi_tx, spi_rx;
    wire       miso, mosi, sclk;

    SD_cmd cmd (
        .clk(clk), .rst(rst),
        .cmd_start(cmd_start), .cmd_index(cmd_index), .cmd_arg(cmd_arg),
        .extra_bytes(extra_bytes),
        .cmd_done(cmd_done), .cmd_busy(cmd_busy), .r1(r1),
        .resp_data(resp_data), .timeout(timeout),
        .spi_start(spi_start), .spi_tx(spi_tx), .spi_rx(spi_rx), .spi_done(spi_done)
    );

    SPI_driver spi (
        .clk(clk), .rst(rst), .fast(1'b1), .fast_half(9'd2),
        .start(spi_start), .tx_data(spi_tx), .rx_data(spi_rx),
        .done(spi_done), .busy(spi_busy),
        .miso(miso), .mosi(mosi), .sclk(sclk)
    );

    // ------------------------------------------------------------------
    // Fake SD card
    //   bytes 0-5 : receives the command, sends 0xFF
    //   then      : sends 0xFF for card_delay bytes, then R1, then extras
    // ------------------------------------------------------------------
    integer    card_delay  = 1;
    reg [7:0]  card_r1     = 8'h01;
    reg [31:0] card_extra  = 32'h0;
    integer    card_nextra = 0;

    reg [7:0]  card_tx = 8'hFF;   // byte being shifted out on MISO
    reg [7:0]  card_rx_shift = 0;
    integer    rise_bits = 0, fall_bits = 0;
    integer    bytes_done = 0;    // full bytes exchanged
    reg [7:0]  got [0:15];        // bytes the card received

    assign miso = card_tx[7];

    function [7:0] card_next(input integer idx);   // byte to send at position idx
        integer k;
        begin
            if (idx < 6) card_next = 8'hFF;
            else begin
                k = idx - 6;
                if      (k <  card_delay)               card_next = 8'hFF;
                else if (k == card_delay)               card_next = card_r1;
                else if (k <= card_delay + card_nextra) card_next = card_extra >> (8 * (card_delay + card_nextra - k));
                else                                    card_next = 8'hFF;
            end
        end
    endfunction

    always @(posedge sclk) begin
        card_rx_shift = {card_rx_shift[6:0], mosi};
        rise_bits = rise_bits + 1;
        if (rise_bits == 8) begin
            if (bytes_done < 16) got[bytes_done] = card_rx_shift;
            bytes_done = bytes_done + 1;
            rise_bits  = 0;
        end
    end

    always @(negedge sclk) begin
        fall_bits = fall_bits + 1;
        if (fall_bits == 8) begin
            card_tx   = card_next(bytes_done);   // next byte ready before next rising edge
            fall_bits = 0;
        end else
            card_tx = {card_tx[6:0], 1'b1};
    end

    task card_reset;
        begin
            bytes_done = 0; rise_bits = 0; fall_bits = 0;
            card_tx = 8'hFF;
        end
    endtask

    // ------------------------------------------------------------------
    // Bookkeeping
    // ------------------------------------------------------------------
    integer pass_count = 0, fail_count = 0;

    task check(input cond, input [8*64-1:0] msg);
        begin
            if (cond) begin pass_count = pass_count + 1; $display("  PASS: %0s", msg); end
            else      begin fail_count = fail_count + 1; $display("  FAIL: %0s   <-----", msg); end
        end
    endtask

    // Run one command and wait for cmd_done
    reg hung;
    integer cycles;
    task run_cmd(input [5:0] idx, input [31:0] arg, input [2:0] nextra);
        begin
            card_reset;
            hung = 0;
            @(negedge clk);
            cmd_index = idx; cmd_arg = arg; extra_bytes = nextra;
            cmd_start = 1;
            @(negedge clk);
            cmd_start = 0;
            cycles = 0;
            while (!cmd_done && cycles < 50000) begin
                @(negedge clk);
                cycles = cycles + 1;
            end
            if (!cmd_done) hung = 1;
            $display("  card received: %02h %02h %02h %02h %02h %02h | R1=%02h resp=%08h timeout=%0d | bytes on bus=%0d",
                     got[0], got[1], got[2], got[3], got[4], got[5], r1, resp_data, timeout, bytes_done);
            @(negedge clk);
        end
    endtask

    task check_frame(input [47:0] f);
        begin
            check({got[0], got[1], got[2], got[3], got[4], got[5]} == f, "command bytes correct (incl. checksum)");
        end
    endtask

    // ------------------------------------------------------------------
    // Tests
    // ------------------------------------------------------------------
    initial begin
        $display("");
        $display("==============================================");
        $display(" SD_cmd testbench");
        $display("==============================================");

        repeat (5) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        check(cmd_busy == 0 && cmd_done == 0, "idle after reset");

        // ---------------- CMD0 ----------------
        $display("");
        $display("[TEST 1] CMD0 - reset, card answers 0x01 after 1 filler byte");
        card_delay = 1; card_r1 = 8'h01; card_nextra = 0;
        run_cmd(6'd0, 32'h0, 3'd0);
        check(!hung,               "command finished");
        check_frame(48'h40_00000000_95);
        check(r1 == 8'h01,         "R1 = 0x01");
        check(!timeout,            "no timeout");
        check(bytes_done == 6 + 2, "stopped right after the reply (8 bytes total)");

        // ---------------- CMD8 ----------------
        $display("");
        $display("[TEST 2] CMD8 - voltage check, 4 extra bytes");
        card_delay = 1; card_r1 = 8'h01; card_extra = 32'h000001AA; card_nextra = 4;
        run_cmd(6'd8, 32'h000001AA, 3'd4);
        check(!hung,                     "command finished");
        check_frame(48'h48_000001AA_87);
        check(r1 == 8'h01,               "R1 = 0x01");
        check(resp_data == 32'h000001AA, "extra bytes = 000001AA");
        check(bytes_done == 6 + 2 + 4,   "read exactly 4 extra bytes");

        // ---------------- CMD55 / ACMD41 ----------------
        $display("");
        $display("[TEST 3] CMD55 then ACMD41");
        card_delay = 0; card_r1 = 8'h01; card_nextra = 0;
        run_cmd(6'd55, 32'h0, 3'd0);
        check_frame(48'h77_00000000_65);
        check(r1 == 8'h01, "CMD55 R1 = 0x01");
        card_r1 = 8'h00;
        run_cmd(6'd41, 32'h40000000, 3'd0);
        check_frame(48'h69_40000000_77);
        check(r1 == 8'h00, "ACMD41 R1 = 0x00 (card ready)");

        // ---------------- CMD58 ----------------
        $display("");
        $display("[TEST 4] CMD58 - read card settings");
        card_delay = 2; card_r1 = 8'h00; card_extra = 32'hC0FF8000; card_nextra = 4;
        run_cmd(6'd58, 32'h0, 3'd4);
        check_frame(48'h7A_00000000_FD);
        check(r1 == 8'h00,               "R1 = 0x00");
        check(resp_data == 32'hC0FF8000, "extra bytes = C0FF8000");
        check(resp_data[31] && resp_data[30], "powered-up and large-card bits set");

        // ---------------- CMD17 ----------------
        $display("");
        $display("[TEST 5] CMD17 - read block 0x1234, argument bytes in right order");
        card_delay = 0; card_r1 = 8'h00; card_nextra = 0;
        run_cmd(6'd17, 32'h00001234, 3'd0);
        check({got[0], got[1], got[2], got[3], got[4]} == 40'h51_00001234, "command + argument bytes correct");
        check(got[5][0] == 1'b1, "end bit is 1");
        check(r1 == 8'h00,       "R1 = 0x00");

        // ---------------- Slowest allowed reply ----------------
        $display("");
        $display("[TEST 6] Reply on the 8th (last allowed) try");
        card_delay = 7; card_r1 = 8'h01; card_nextra = 0;
        run_cmd(6'd0, 32'h0, 3'd0);
        check(r1 == 8'h01 && !timeout, "reply accepted on try 8");

        // ---------------- Timeout ----------------
        $display("");
        $display("[TEST 7] Card never answers - should time out after 8 tries");
        card_delay = 100;
        run_cmd(6'd0, 32'h0, 3'd0);
        check(!hung,               "engine did not hang");
        check(timeout,             "timeout flag set");
        check(bytes_done == 6 + 8, "gave up after exactly 8 tries");

        // ---------------- Recovery ----------------
        $display("");
        $display("[TEST 8] Normal command after a timeout");
        card_delay = 1; card_r1 = 8'h01;
        run_cmd(6'd0, 32'h0, 3'd0);
        check(r1 == 8'h01 && !timeout, "timeout flag cleared, works again");

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

    initial begin
        #10_000_000;
        $display("GLOBAL TIMEOUT - simulation hung");
        $finish;
    end

endmodule