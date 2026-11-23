`timescale 1ns / 1ps
// Self-checking testbench for SD_controller (+ SD_cmd + SPI_driver)
//
// The fake SD card:
//   - counts clocks with CS high before the first command (needs >= 74)
//   - decodes every 6-byte command and logs it
//   - answers ACMD41 with "busy" a few times before "ready"
//   - answers CMD17 with a few 0xFF, then 0xFE, 512 data bytes, 2 checksum bytes
//   - has fault modes to test every error path
//
// Data pattern for block N, byte i:  (N[7:0] + i) & 0xFF

module SD_controller_tb;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;   // 100 MHz

    initial begin
        // Top-level signals plus the controller's state only.
        // Dumping everything makes a ~250 MB file (the timers toggle every clock).
        $dumpfile("sd_controller.vcd");
        $dumpvars(1, SD_controller_tb);
        $dumpvars(0, SD_controller_tb.dut.state);
        $dumpvars(0, SD_controller_tb.dut.use_cmd);
    end

    // ---------------- DUT ----------------
    reg         reinit = 0;
    reg         start_read = 0;
    reg  [31:0] block_addr = 0;
    wire        init_done, busy, error;
    wire [3:0]  err_code;
    wire [7:0]  data_out;
    wire        data_valid;
    wire [8:0]  byte_index;
    wire        block_done;
    wire        sclk, mosi, cs_n;
    wire        miso;
    wire [15:0] dbg_cmd0, dbg_cmd8, dbg_acmd41, dbg_ocr, dbg_read, dbg_polls;

    // Timeouts shortened so the simulation runs quickly
    SD_controller #(
        .POWER_WAIT_CYCLES(1_000),     // 10 us instead of 1 ms
        .ACMD41_TIMEOUT(500_000),      // 5 ms instead of 1 s
        .TOKEN_TIMEOUT(20_000)         // 200 us instead of 100 ms
    ) dut (
        .clk(clk), .rst(rst),
        .reinit(reinit), .start_read(start_read), .block_addr(block_addr), .fast_half(9'd2),
        .init_done(init_done), .busy(busy), .error(error), .err_code(err_code),
        .data_out(data_out), .data_valid(data_valid), .byte_index(byte_index),
        .block_done(block_done),
        .dbg_cmd0(dbg_cmd0), .dbg_cmd8(dbg_cmd8), .dbg_acmd41(dbg_acmd41),
        .dbg_ocr(dbg_ocr), .dbg_read(dbg_read), .dbg_polls(dbg_polls),
        .sclk(sclk), .mosi(mosi), .miso(miso), .cs_n(cs_n)
    );

    // ==================================================================
    // FAKE SD CARD
    // ==================================================================
    // fault modes
    reg     card_dead        = 0;  // never answers anything
    integer cmd0_ignore      = 0;  // ignore this many CMD0s first (like real cards)
    integer cmd0_seen        = 0;
    reg     acmd41_never_rdy = 0;  // ACMD41 always says busy
    reg     no_token         = 0;  // CMD17 accepted but 0xFE never comes
    reg     bad_token        = 0;  // sends error marker 0x05 instead of 0xFE
    reg     reject_cmd17     = 0;  // CMD17 answered with address error 0x20
    integer acmd41_busy_count = 3; // how many times ACMD41 says busy

    // outgoing byte queue
    reg [7:0] q [0:2047];
    integer   q_head = 0, q_tail = 0;

    task q_push(input [7:0] b);
        begin q[q_tail % 2048] = b; q_tail = q_tail + 1; end
    endtask

    // bit-level SPI slave
    reg [7:0] card_tx = 8'hFF;
    reg [7:0] rx_shift = 8'h00;
    integer   rise_bits = 0, fall_bits = 0;
    assign miso = cs_n ? 1'b1 : card_tx[7];

    // command decoder
    reg [7:0]  cmd_buf [0:5];
    integer    cmd_pos = 0;
    integer    acmd41_seen = 0;
    reg        last_was_55 = 0;

    // logs
    integer    wake_clocks = 0;       // SCLK rising edges with CS high before first command
    reg        first_cmd_seen = 0;
    integer    cmd_log [0:63];
    integer    cmd_log_n = 0;
    reg        crc_ok_0 = 0, crc_ok_8 = 0;
    realtime   last_rise = 0, sclk_period = 0;
    reg [31:0] last_cmd17_arg = 0;

    task card_reset;
        begin
            q_head = 0; q_tail = 0;
            cmd_pos = 0; acmd41_seen = 0; last_was_55 = 0; cmd0_seen = 0;
            wake_clocks = 0; first_cmd_seen = 0; cmd_log_n = 0;
            card_tx = 8'hFF; rise_bits = 0; fall_bits = 0;
        end
    endtask

    task handle_command;
        reg [5:0]  idx;
        reg [31:0] arg;
        reg [7:0]  crc;
        integer    i;
        begin
            idx = cmd_buf[0][5:0];
            arg = {cmd_buf[1], cmd_buf[2], cmd_buf[3], cmd_buf[4]};
            crc = cmd_buf[5];
            first_cmd_seen = 1;
            if (cmd_log_n < 64) begin cmd_log[cmd_log_n] = idx; cmd_log_n = cmd_log_n + 1; end

            if (idx == 6'd0) cmd0_seen = cmd0_seen + 1;
            if (!card_dead && !(idx == 6'd0 && cmd0_seen <= cmd0_ignore)) begin
                q_push(8'hFF);   // one "thinking" byte before every reply
                case (idx)
                    6'd0: begin
                        crc_ok_0 = (crc == 8'h95);
                        q_push(8'h01);
                    end
                    6'd8: begin
                        crc_ok_8 = (crc == 8'h87);
                        q_push(8'h01);
                        q_push(8'h00); q_push(8'h00);
                        q_push({4'h0, arg[11:8]}); q_push(arg[7:0]);   // echo
                    end
                    6'd55: q_push(8'h01);
                    6'd41: begin
                        acmd41_seen = acmd41_seen + 1;
                        if (acmd41_never_rdy || acmd41_seen <= acmd41_busy_count)
                            q_push(8'h01);   // still busy
                        else
                            q_push(8'h00);   // ready
                    end
                    6'd58: begin
                        q_push(8'h00);
                        q_push(8'hC0); q_push(8'hFF); q_push(8'h80); q_push(8'h00);
                    end
                    6'd17: begin
                        last_cmd17_arg = arg;
                        if (reject_cmd17) q_push(8'h20);
                        else begin
                            q_push(8'h00);
                            if (!no_token) begin
                                q_push(8'hFF); q_push(8'hFF); q_push(8'hFF);  // card "looking up"
                                if (bad_token) q_push(8'h05);
                                else begin
                                    q_push(8'hFE);
                                    for (i = 0; i < 512; i = i + 1)
                                        q_push(arg[7:0] + i[7:0]);
                                    q_push(8'hAB); q_push(8'hCD);      // checksum bytes
                                end
                            end
                        end
                    end
                    default: q_push(8'h04);  // illegal command
                endcase
            end
        end
    endtask

    // CS going low: start fresh byte alignment
    always @(negedge cs_n) begin
        rise_bits = 0; fall_bits = 0;
        card_tx = (q_head < q_tail) ? q[q_head % 2048] : 8'hFF;
        if (q_head < q_tail) q_head = q_head + 1;
    end

    always @(posedge sclk) begin
        if (last_rise != 0) sclk_period = $realtime - last_rise;
        last_rise = $realtime;

        if (cs_n) begin
            if (!first_cmd_seen) wake_clocks = wake_clocks + 1;
        end else begin
            rx_shift  = {rx_shift[6:0], mosi};
            rise_bits = rise_bits + 1;
            if (rise_bits == 8) begin
                rise_bits = 0;
                if (cmd_pos == 0 && rx_shift[7:6] == 2'b01) begin
                    cmd_buf[0] = rx_shift; cmd_pos = 1;
                end else if (cmd_pos > 0) begin
                    cmd_buf[cmd_pos] = rx_shift;
                    cmd_pos = cmd_pos + 1;
                    if (cmd_pos == 6) begin
                        cmd_pos = 0;
                        handle_command;
                    end
                end
            end
        end
    end

    always @(negedge sclk) begin
        if (!cs_n) begin
            fall_bits = fall_bits + 1;
            if (fall_bits == 8) begin
                fall_bits = 0;
                if (q_head < q_tail) begin
                    card_tx = q[q_head % 2048];
                    q_head  = q_head + 1;
                end else
                    card_tx = 8'hFF;
            end else
                card_tx = {card_tx[6:0], 1'b1};
        end
    end

    // ==================================================================
    // DATA MONITOR: checks every streamed byte
    // ==================================================================
    integer valid_count = 0;
    integer data_errors = 0;
    integer index_errors = 0;
    integer done_count = 0;
    reg [31:0] expect_block = 0;

    always @(posedge clk) begin
        if (data_valid) begin
            if (byte_index != valid_count[8:0]) index_errors = index_errors + 1;
            if (data_out != ((expect_block[7:0] + valid_count) & 8'hFF)) data_errors = data_errors + 1;
            valid_count = valid_count + 1;
        end
        if (block_done) done_count = done_count + 1;
    end

    // ==================================================================
    // Bookkeeping + helpers
    // ==================================================================
    integer pass_count = 0, fail_count = 0;

    task check(input cond, input [8*72-1:0] msg);
        begin
            if (cond) begin pass_count = pass_count + 1; $display("  PASS: %0s", msg); end
            else      begin fail_count = fail_count + 1; $display("  FAIL: %0s   <-----", msg); end
        end
    endtask

    task print_cmd_log;
        integer i;
        begin
            $write("  commands the card saw:");
            for (i = 0; i < cmd_log_n; i = i + 1) $write(" CMD%0d", cmd_log[i]);
            $write("\n");
        end
    endtask

    // wait until init_done or error (or give up)
    integer cycles;
    task wait_init;
        begin
            cycles = 0;
            while (!init_done && !error && cycles < 3_000_000) begin
                @(negedge clk); cycles = cycles + 1;
            end
        end
    endtask

    task pulse_reinit;
        begin
            card_reset;
            @(negedge clk); reinit = 1;
            @(negedge clk); reinit = 0;
        end
    endtask

    // read one block, wait for block_done or error
    task read_block(input [31:0] addr);
        begin
            valid_count = 0; data_errors = 0; index_errors = 0; done_count = 0;
            expect_block = addr;
            @(negedge clk);
            block_addr = addr; start_read = 1;
            @(negedge clk);
            start_read = 0;
            cycles = 0;
            while (!block_done && !error && cycles < 500_000) begin
                @(negedge clk); cycles = cycles + 1;
            end
            repeat (3) @(negedge clk);
        end
    endtask

    // ==================================================================
    // TESTS
    // ==================================================================
    integer i;
    reg     seq_ok;

    initial begin
        $display("");
        $display("==============================================");
        $display(" SD_controller testbench");
        $display("==============================================");

        card_reset;
        repeat (5) @(negedge clk);
        rst = 0;

        // ---------------- TEST 1: startup ----------------
        $display("");
        $display("[TEST 1] Startup sequence (card busy 3 times on ACMD41)");
        check(cs_n == 1'b1, "CS high right after reset");
        wait_init;
        print_cmd_log;
        $display("  wake-up clocks with CS high = %0d", wake_clocks);
        check(init_done && !error,  "init_done high, no error");
        check(wake_clocks >= 74,    "at least 74 wake-up clocks with CS high");
        check(crc_ok_0,             "CMD0 checksum was 0x95");
        check(crc_ok_8,             "CMD8 checksum was 0x87");

        seq_ok = (cmd_log_n == 11);   // 3 busy pairs + 1 ready pair = 4 pairs
        // expected: 0, 8, (55,41) x4, 58
        if (seq_ok) begin
            seq_ok = (cmd_log[0] == 0) && (cmd_log[1] == 8) && (cmd_log[10] == 58);
            for (i = 0; i < 4; i = i + 1)
                seq_ok = seq_ok && (cmd_log[2 + 2*i] == 55) && (cmd_log[3 + 2*i] == 41);
        end
        check(seq_ok, "commands in order: 0, 8, (55,41) x4, 58");
        check(cs_n == 1'b1 && !busy, "CS high and not busy after startup");
        $display("  replies: CMD0=%04h CMD8=%04h ACMD41=%04h OCR=%04h", dbg_cmd0, dbg_cmd8, dbg_acmd41, dbg_ocr);
        check(dbg_cmd0   == 16'h0101, "CMD0 reply captured: 1 try, R1 = 01");
        check(dbg_cmd8   == 16'h01AA, "CMD8 echo captured: 01AA");
        check(dbg_acmd41 == 16'h0400, "ACMD41 captured: 4 tries, final R1 = 00");
        check(dbg_ocr    == 16'hC0FF, "CMD58 OCR captured: C0FF");

        // ---------------- TEST 2: read a block ----------------
        $display("");
        $display("[TEST 2] Read block 5");
        read_block(32'd5);
        $display("  data bytes=%0d  wrong data=%0d  wrong index=%0d  block_done pulses=%0d  SCLK period=%0.1f ns",
                 valid_count, data_errors, index_errors, done_count, sclk_period);
        check(!error,                        "no error");
        check(last_cmd17_arg == 32'd5,       "CMD17 sent with block address 5");
        check(valid_count == 512,            "exactly 512 data_valid pulses");
        check(data_errors == 0,              "every data byte correct");
        check(index_errors == 0,             "byte_index counted 0 to 511");
        check(done_count == 1,               "block_done pulsed once");
        check(sclk_period > 39.0 && sclk_period < 41.0, "running at 25 MHz after startup");
        check(cs_n == 1'b1 && !busy,         "CS high and back to idle");
        $display("  read reply=%04h polls=%0d", dbg_read, dbg_polls);
        check(dbg_read  == 16'h00FE,         "CMD17 reply captured: R1 = 00, token = FE");
        check(dbg_polls == 16'd4,            "polls captured: 3 x FF then FE = 4");

        // ---------------- TEST 3: another block ----------------
        $display("");
        $display("[TEST 3] Read block 0x1234 right after");
        read_block(32'h0000_1234);
        $display("  data bytes=%0d  wrong data=%0d  wrong index=%0d  block_done pulses=%0d",
                 valid_count, data_errors, index_errors, done_count);
        check(last_cmd17_arg == 32'h1234,    "CMD17 sent with block address 0x1234");
        check(valid_count == 512 && data_errors == 0 && index_errors == 0, "all 512 bytes correct");
        check(done_count == 1,               "block_done pulsed once");

        // ---------------- TEST 4: CMD17 rejected ----------------
        $display("");
        $display("[TEST 4] Card rejects CMD17 (address error)");
        reject_cmd17 = 1;
        read_block(32'd7);
        $display("  error=%0d err_code=%0d", error, err_code);
        check(error && err_code == 4'd5, "error code 5 (CMD17 rejected)");
        check(valid_count == 0,          "no data bytes sent out");
        reject_cmd17 = 0;

        // ---------------- TEST 5: recover with reinit ----------------
        $display("");
        $display("[TEST 5] reinit after an error");
        pulse_reinit;
        wait_init;
        check(init_done && !error && err_code == 0, "startup ran again, error cleared");
        read_block(32'd9);
        check(valid_count == 512 && data_errors == 0 && done_count == 1, "block read works again");

        // ---------------- TEST 6: 0xFE never comes ----------------
        $display("");
        $display("[TEST 6] Card never sends 0xFE");
        no_token = 1;
        read_block(32'd2);
        $display("  error=%0d err_code=%0d", error, err_code);
        check(error && err_code == 4'd6, "error code 6 (token timeout)");
        no_token = 0;

        // ---------------- TEST 7: error token ----------------
        $display("");
        $display("[TEST 7] Card sends an error marker instead of 0xFE");
        pulse_reinit; wait_init;
        bad_token = 1;
        read_block(32'd3);
        $display("  error=%0d err_code=%0d", error, err_code);
        check(error && err_code == 4'd7, "error code 7 (error marker)");
        bad_token = 0;

        // ---------------- TEST 8: ACMD41 never ready ----------------
        $display("");
        $display("[TEST 8] Card stays busy forever on ACMD41");
        acmd41_never_rdy = 1;
        pulse_reinit; wait_init;
        $display("  error=%0d err_code=%0d  ACMD41 attempts=%0d", error, err_code, acmd41_seen);
        check(error && err_code == 4'd3, "error code 3 (ACMD41 timeout)");
        check(!init_done,                "init_done stays low");
        acmd41_never_rdy = 0;

        // ---------------- TEST 9: dead card ----------------
        $display("");
        $display("[TEST 9] No card / card not answering");
        card_dead = 1;
        pulse_reinit; wait_init;
        $display("  error=%0d err_code=%0d", error, err_code);
        check(error && err_code == 4'd1, "error code 1 (CMD0 failed)");
        check(dbg_cmd0 == 16'h14FF,      "CMD0 capture shows 20 tries (hex 14), no reply (FF)");
        card_dead = 0;

        // ---------------- TEST 10: start_read ignored before init ----------------
        $display("");
        $display("[TEST 10] start_read ignored while in error / not initialized");
        @(negedge clk); start_read = 1; @(negedge clk); start_read = 0;
        repeat (100) @(negedge clk);
        check(error && !busy && cs_n, "stayed in error, nothing sent");

        // ---------------- TEST 10b: card ignores the first CMD0s ----------------
        $display("");
        $display("[TEST 10b] Card ignores the first 3 CMD0s (common on real cards)");
        cmd0_ignore = 3;
        pulse_reinit; wait_init;
        $display("  CMD0 attempts=%0d  init_done=%0d error=%0d", cmd0_seen, init_done, error);
        check(init_done && !error && cmd0_seen == 4, "retried CMD0 and started up on try 4");
        check(dbg_cmd0 == 16'h0401,                "CMD0 capture shows 4 tries, R1 = 01");
        cmd0_ignore = 0;

        // ---------------- TEST 11: full recovery ----------------
        $display("");
        $display("[TEST 11] Final recovery");
        pulse_reinit; wait_init;
        read_block(32'd42);
        check(init_done && !error, "startup OK");
        check(valid_count == 512 && data_errors == 0 && done_count == 1, "block 42 read correctly");

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
        #200_000_000;
        $display("GLOBAL TIMEOUT - simulation hung");
        $finish;
    end

endmodule
