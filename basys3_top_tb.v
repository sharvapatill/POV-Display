`timescale 1ns / 1ps
// Board-level testbench for basys3_top: drives the switches and buttons like a person would,
// with an SPI-level fake SD card on the Pmod pins. Checks LEDs, every display view,
// the real 7-segment pin patterns, replay, slow SPI, and card/file faults.
// Self-contained (no .hex files), so it runs as-is in Vivado (run all) or iverilog.
//
// Fake card (FAT32):
//   block 0              = MBR, partition 1 at block 2048
//   block 2048           = boot record: 512 B/sector, 8 sectors/cluster, 32 reserved, 2 FATs x 7600, root cluster 2
//   block 2080           = FAT: 2 = end (root dir), 3 -> 4 -> 5 = end (VIDEO.BIN)
//   block 17280          = root dir (cluster 2): VIDEO.BIN, cluster 3, 10000 bytes
//   blocks 17288-17311   = VIDEO.BIN data (clusters 3-5), pattern byte i = i[7:0]^i[15:8]^i[23:16]
// Blocks at or past 0x01000000 are rejected with R1 = 0x40, like a real card.
//
// Fault knobs (set by the tests): card_present, bad_mbr_sig, name_first, corrupt_off.

module basys3_top_tb;

    reg clk = 0;
    always #5 clk = ~clk;

    reg  [15:0] sw = 0;
    reg         btnC = 0, btnU = 0;
    wire [15:0] led;
    wire [6:0]  seg;
    wire        dp;
    wire [3:0]  an;
    wire        sd_cs_n, sd_mosi, sd_sclk;
    wire        sd_miso;

    basys3_top dut (
        .clk(clk), .btnC(btnC), .btnU(btnU), .sw(sw), .led(led),
        .seg(seg), .dp(dp), .an(an),
        .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi), .sd_miso(sd_miso), .sd_sclk(sd_sclk)
    );

    // ==================================================================
    // Disk contents
    // ==================================================================
    localparam [31:0] PART       = 32'd2048;
    localparam [31:0] FAT_LBA    = 32'd2080;    // PART + 32 reserved
    localparam [31:0] DATA_LBA   = 32'd17280;   // FAT_LBA + 2 * 7600
    localparam [31:0] FILE_LBA   = 32'd17288;   // cluster 3
    localparam integer FILE_SIZE = 10000;       // 0x2710: 20 sectors over 3 clusters

    reg        card_present = 1'b1;
    reg        bad_mbr_sig  = 1'b0;
    reg [7:0]  name_first   = "V";               // first letter of the file name on the card
    integer    corrupt_off  = -1;                // file byte the card returns inverted (-1 = none)

    reg [87:0] fname = "VIDEO   BIN";

    function [7:0] pattern(input integer off);
        pattern = off[7:0] ^ off[15:8] ^ off[23:16];
    endfunction

    function [31:0] fat_entry(input integer c);
        case (c)
            0:       fat_entry = 32'h0FFF_FFF8;
            1, 2, 5: fat_entry = 32'h0FFF_FFFF;
            3:       fat_entry = 32'd4;
            4:       fat_entry = 32'd5;
            default: fat_entry = 32'd0;
        endcase
    endfunction

    function [7:0] disk_byte(input [31:0] blk, input integer i);
        integer off;
        reg [31:0] v;
        begin
            disk_byte = 8'h00;
            if (blk == 0) begin                                     // MBR
                case (i)
                    450: disk_byte = 8'h0C;
                    454: disk_byte = PART[7:0];
                    455: disk_byte = PART[15:8];
                    510: disk_byte = bad_mbr_sig ? 8'h00 : 8'h55;
                    511: disk_byte = bad_mbr_sig ? 8'h00 : 8'hAA;
                    default: ;
                endcase
            end else if (blk == PART) begin                         // boot record
                case (i)
                    0: disk_byte = 8'hEB;  1: disk_byte = 8'h58;  2: disk_byte = 8'h90;
                    12: disk_byte = 8'h02;                          // 512 bytes/sector
                    13: disk_byte = 8'd8;                           // sectors/cluster
                    14: disk_byte = 8'd32;                          // reserved
                    16: disk_byte = 8'd2;                           // FATs
                    36: disk_byte = 8'hB0; 37: disk_byte = 8'h1D;   // 7600 sectors per FAT
                    44: disk_byte = 8'd2;                           // root cluster
                    510: disk_byte = 8'h55; 511: disk_byte = 8'hAA;
                    default: ;
                endcase
            end else if (blk == FAT_LBA) begin                      // FAT sector 0
                v = fat_entry(i / 4);
                disk_byte = v >> (8 * (i % 4));
            end else if (blk == DATA_LBA) begin                     // root directory
                if (i < 11)       disk_byte = (i == 0) ? name_first : fname[87 - 8*i -: 8];
                else if (i == 11) disk_byte = 8'h20;                // archive
                else if (i == 26) disk_byte = 8'd3;                 // first cluster = 3
                else if (i >= 28 && i < 32) disk_byte = FILE_SIZE >> (8 * (i - 28));
            end else if (blk >= FILE_LBA && blk < FILE_LBA + 24) begin  // VIDEO.BIN
                off = (blk - FILE_LBA) * 512 + i;
                if (off >= FILE_SIZE)      disk_byte = 8'hEE;       // slack after the end
                else if (off == corrupt_off) disk_byte = ~pattern(off);
                else                       disk_byte = pattern(off);
            end
        end
    endfunction

    // expected XOR checksum of the file
    reg [7:0] file_xor;
    integer   n;
    initial begin
        file_xor = 8'h00;
        for (n = 0; n < FILE_SIZE; n = n + 1) file_xor = file_xor ^ pattern(n);
    end

    // ==================================================================
    // Fake SD card (SPI slave)
    // ==================================================================
    reg [7:0] q [0:2047];
    integer   q_head = 0, q_tail = 0;
    task q_push(input [7:0] b); begin q[q_tail % 2048] = b; q_tail = q_tail + 1; end endtask

    reg [7:0] card_tx = 8'hFF, rx_shift = 8'h00;
    integer   rise_bits = 0, fall_bits = 0;
    assign sd_miso = (sd_cs_n || !card_present) ? 1'b1 : card_tx[7];   // pull-up when idle or empty

    reg [7:0]  cmd_buf [0:5];
    integer    cmd_pos = 0, acmd41_seen = 0, reads = 0;
    reg [31:0] last_read = 0;

    task card_reset; begin q_head = 0; q_tail = 0; cmd_pos = 0; acmd41_seen = 0; card_tx = 8'hFF; end endtask

    task handle_command;
        reg [5:0] idx; reg [31:0] arg; integer i;
        begin
            idx = cmd_buf[0][5:0];
            arg = {cmd_buf[1], cmd_buf[2], cmd_buf[3], cmd_buf[4]};
            q_push(8'hFF);
            case (idx)
                6'd0:  q_push(8'h01);
                6'd8:  begin q_push(8'h01); q_push(8'h00); q_push(8'h00); q_push({4'h0, arg[11:8]}); q_push(arg[7:0]); end
                6'd55: q_push(8'h01);
                6'd41: begin acmd41_seen = acmd41_seen + 1; q_push(acmd41_seen <= 2 ? 8'h01 : 8'h00); end
                6'd58: begin q_push(8'h00); q_push(8'hC0); q_push(8'hFF); q_push(8'h80); q_push(8'h00); end
                6'd17: begin
                    last_read = arg;
                    reads = reads + 1;
                    if (arg >= 32'h0100_0000) q_push(8'h40);
                    else begin
                        q_push(8'h00); q_push(8'hFF); q_push(8'hFF); q_push(8'hFE);
                        for (i = 0; i < 512; i = i + 1) q_push(disk_byte(arg, i));
                        q_push(8'hAB); q_push(8'hCD);
                    end
                end
                default: q_push(8'h04);
            endcase
        end
    endtask

    always @(negedge sd_cs_n) begin
        rise_bits = 0; fall_bits = 0;
        card_tx = (q_head < q_tail) ? q[q_head % 2048] : 8'hFF;
        if (q_head < q_tail) q_head = q_head + 1;
    end
    always @(posedge sd_sclk) if (!sd_cs_n && card_present) begin
        rx_shift = {rx_shift[6:0], sd_mosi}; rise_bits = rise_bits + 1;
        if (rise_bits == 8) begin
            rise_bits = 0;
            if (cmd_pos == 0 && rx_shift[7:6] == 2'b01) begin cmd_buf[0] = rx_shift; cmd_pos = 1; end
            else if (cmd_pos > 0) begin
                cmd_buf[cmd_pos] = rx_shift; cmd_pos = cmd_pos + 1;
                if (cmd_pos == 6) begin cmd_pos = 0; handle_command; end
            end
        end
    end
    always @(negedge sd_sclk) if (!sd_cs_n) begin
        fall_bits = fall_bits + 1;
        if (fall_bits == 8) begin
            fall_bits = 0;
            if (q_head < q_tail) begin card_tx = q[q_head % 2048]; q_head = q_head + 1; end
            else card_tx = 8'hFF;
        end else card_tx = {card_tx[6:0], 1'b1};
    end

    // SCLK period (the last bit measured is always inside a byte)
    realtime last_rise = 0, sclk_period = 0;
    always @(posedge sd_sclk) begin
        if (last_rise != 0) sclk_period = $realtime - last_rise;
        last_rise = $realtime;
    end

    // ==================================================================
    // Helpers
    // ==================================================================
    integer pass_count = 0, fail_count = 0;
    task check(input cond, input [8*72-1:0] msg);
        begin
            if (cond) begin pass_count = pass_count + 1; $display("  PASS: %0s", msg); end
            else      begin fail_count = fail_count + 1; $display("  FAIL: %0s   <-----", msg); end
        end
    endtask

    // LED groups, named like the board legend
    wire       L_found = led[0], L_fat_err = led[1], L_done = led[2];
    wire [3:0] L_loc   = led[6:3];
    wire [1:0] L_rdr   = led[8:7];
    wire       L_ready = led[9], L_sd_err = led[10];
    wire [3:0] L_sd    = led[14:11];

    task show_leds;
        $display("  LEDs: found=%0d fat_err=%0d done=%0d loc_err=%0d rdr_err=%0d | ready=%0d sd_err=%0d sd_code=%0d",
                 L_found, L_fat_err, L_done, L_loc, L_rdr, L_ready, L_sd_err, L_sd);
    endtask

    // wait until the file is done or anything reports an error
    integer cycles;
    task wait_finished;
        begin
            cycles = 0;
            while (!L_done && !L_fat_err && !L_sd_err && cycles < 20_000_000) begin
                @(negedge clk); cycles = cycles + 1;
            end
            repeat (200) @(negedge clk);
            show_leds;
        end
    endtask

    // press btnC (reset; SW15 is read at this moment), then wait
    task press_reset_and_wait;
        begin
            card_reset;
            reads = 0;
            @(negedge clk); btnC = 1;
            repeat (20) @(negedge clk);
            btnC = 0;
            repeat (10) @(negedge clk);
            wait_finished;
        end
    endtask

    // hold btnU past the 10 ms debounce, release, then wait
    task press_replay_and_wait;
        begin
            reads = 0;
            @(negedge clk); btnU = 1;
            repeat (1_200_000) @(negedge clk);
            btnU = 0;
            repeat (1_200_000) @(negedge clk);
            wait_finished;
        end
    endtask

    // put one view switch up (SW0-SW13) and read the value going to the display
    task view(input integer n, input [15:0] expect, input [8*40-1:0] name);
        begin
            sw[13:0] = 14'd0;
            if (n >= 0) sw[n] = 1'b1;
            repeat (5) @(negedge clk);
            $display("  view SW%0d %0s = %04h", n, name, dut.disp);
            check(dut.disp == expect, name);
            sw[13:0] = 14'd0;
        end
    endtask

    // decode what the 7-segment pins are showing
    function [3:0] seg_to_hex(input [6:0] s);
        case (s)
            7'b1000000: seg_to_hex = 4'h0; 7'b1111001: seg_to_hex = 4'h1; 7'b0100100: seg_to_hex = 4'h2;
            7'b0110000: seg_to_hex = 4'h3; 7'b0011001: seg_to_hex = 4'h4; 7'b0010010: seg_to_hex = 4'h5;
            7'b0000010: seg_to_hex = 4'h6; 7'b1111000: seg_to_hex = 4'h7; 7'b0000000: seg_to_hex = 4'h8;
            7'b0010000: seg_to_hex = 4'h9; 7'b0001000: seg_to_hex = 4'hA; 7'b0000011: seg_to_hex = 4'hB;
            7'b1000110: seg_to_hex = 4'hC; 7'b0100001: seg_to_hex = 4'hD; 7'b0000110: seg_to_hex = 4'hE;
            default:    seg_to_hex = 4'hF;
        endcase
    endfunction
    reg [15:0] shown;
    reg [3:0]  seen;
    task read_display;   // waits for all four digits to be scanned, rebuilds the 4-digit value
        begin
            seen = 0; shown = 0;
            while (seen != 4'b1111) begin
                @(negedge clk);
                case (an)
                    4'b1110: begin shown[3:0]   = seg_to_hex(seg); seen[0] = 1; end
                    4'b1101: begin shown[7:4]   = seg_to_hex(seg); seen[1] = 1; end
                    4'b1011: begin shown[11:8]  = seg_to_hex(seg); seen[2] = 1; end
                    4'b0111: begin shown[15:12] = seg_to_hex(seg); seen[3] = 1; end
                    default: ;
                endcase
            end
        end
    endtask

    // ==================================================================
    // TESTS
    // ==================================================================
    initial begin
        $display("");
        $display("==============================================");
        $display(" basys3_top board-level testbench");
        $display("==============================================");

        // ---------------- Normal run ----------------
        $display("");
        $display("[NORMAL] all switches down, press btnC");
        press_reset_and_wait;
        check(led[14:0] == 15'b0000_0_1_00_0000_1_0_1, "LD0 found + LD2 done + LD9 ready, no errors");
        check(reads == 3 + 22,                         "MBR + boot + root, then 20 data + 2 FAT reads");
        check(sclk_period > 99.0 && sclk_period < 101.0, "SPI at 10 MHz (100 ns period)");
        $display("  SCLK period = %0.1f ns", sclk_period);
        view(-1, 16'h0800,          "part_start");
        view(0,  16'h0003,          "first_cluster");
        view(1,  {8'h00, file_xor}, "XOR checksum");
        view(2,  16'h2710,          "bytes received");
        view(3,  16'h2710,          "file size");
        view(4,  16'h0101,          "CMD0 tries/R1");
        view(5,  16'h01AA,          "CMD8 echo");
        view(6,  16'h0300,          "ACMD41 tries/R1");
        view(7,  16'hC0FF,          "CMD58 OCR upper");
        view(8,  16'h00FE,          "CMD17 R1/token");
        view(9,  16'h0003,          "bytes before token");
        view(10, 16'h0000,          "pattern: bad bytes");
        view(11, 16'h0000,          "pattern: first bad offset");
        view(13, 16'h0000,          "pattern: expected/received");

        // priority when several switches are up
        sw[1] = 1; sw[4] = 1; repeat (5) @(negedge clk);
        check(dut.disp == 16'h0101, "SW1 + SW4 up: SD debug view (SW4) wins");
        sw[13:0] = 0;
        sw[2] = 1; sw[10] = 1; repeat (5) @(negedge clk);
        check(dut.disp == 16'h0000, "SW2 + SW10 up: pattern view (SW10) wins");
        sw[13:0] = 0;

        // the real pins: scan the 7-segment display
        sw[5] = 1;
        read_display;
        $display("  7-segment pins show %04h", shown);
        check(shown == 16'h01AA, "7-segment pins actually show 01AA");
        check(dp == 1'b1,        "decimal point off (value fits in 16 bits)");
        sw[5] = 0;

        // ---------------- Replay ----------------
        $display("");
        $display("[REPLAY] press btnU");
        press_replay_and_wait;
        check(L_found && L_done && !L_fat_err && !L_sd_err, "file streamed again, no errors");
        check(reads == 22,                                  "replay re-reads only the file (20 data + 2 FAT)");
        check(dut.byte_count == FILE_SIZE && dut.checksum == file_xor && dut.bad_count == 0,
              "same byte count, checksum, 0 bad bytes");

        // ---------------- SW15: slow SPI ----------------
        $display("");
        $display("[SW15] up while pressing btnC: SPI at 2.5 MHz");
        sw[15] = 1;
        press_reset_and_wait;
        $display("  SCLK period = %0.1f ns", sclk_period);
        check(sclk_period > 399.0 && sclk_period < 401.0, "SPI really at 2.5 MHz (400 ns period)");
        check(led[14:0] == 15'b0000_0_1_00_0000_1_0_1,   "file still read correctly");
        view(10, 16'h0000, "pattern: bad bytes");
        sw[15] = 0;

        // ---------------- No card ----------------
        $display("");
        $display("[NO CARD] nothing answers on MISO");
        card_present = 1'b0;
        press_reset_and_wait;
        check(led[14:0] == 15'b0001_1_0_00_0000_0_0_0, "only LD10 + LD11: SD error 1");
        view(4, 16'h14FF, "CMD0: 20 tries, no reply (FF)");
        card_present = 1'b1;

        // ---------------- One bad byte ----------------
        $display("");
        $display("[BAD BYTE] card returns file byte 5000 inverted");
        corrupt_off = 5000;                                  // 0x1388, pattern 9B -> 64
        press_reset_and_wait;
        check(L_found && L_done && !L_fat_err,  "file still completes (FAT32 can't tell)");
        view(10, 16'h0001, "pattern: 1 bad byte");
        view(11, 16'h1388, "pattern: at offset 0x1388 = 5000");
        view(12, 16'h0000, "pattern: offset high bits");
        view(13, 16'h9B64, "pattern: expected 9B, got 64");
        corrupt_off = -1;

        // ---------------- Bad MBR ----------------
        $display("");
        $display("[BAD MBR] block 0 has no 55 AA signature");
        bad_mbr_sig = 1'b1;
        press_reset_and_wait;
        check(L_fat_err && L_loc == 4'd2 && !L_sd_err && L_ready, "LD1 + locate_err 2 (LD4), card itself fine");
        check(reads == 1,                                          "stopped after reading block 0");
        bad_mbr_sig = 1'b0;

        // ---------------- File not found ----------------
        $display("");
        $display("[NOT FOUND] root folder has XIDEO.BIN instead of VIDEO.BIN");
        name_first = "X";
        press_reset_and_wait;
        check(L_fat_err && L_loc == 4'd6 && !L_found, "LD1 + locate_err 6 (LD4 + LD5)");
        name_first = "V";

        // ---------------- Back to normal ----------------
        $display("");
        $display("[NORMAL AGAIN] all switches down, press btnC");
        press_reset_and_wait;
        check(led[14:0] == 15'b0000_0_1_00_0000_1_0_1, "found, done, ready, no errors");
        view(1, {8'h00, file_xor}, "XOR checksum");

        $display("");
        $display("==============================================");
        $display(" RESULTS: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display(" ALL TESTS PASSED");
        else                 $display(" SOME TESTS FAILED - see lines marked <-----");
        $display("==============================================");
        $finish;
    end

    initial begin #1_000_000_000; $display("GLOBAL TIMEOUT"); $finish; end

endmodule
