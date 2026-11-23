`timescale 1ns / 1ps
// Board-level testbench for basys3_top: drives the switches and buttons like a person would,
// with an SPI-level fake SD card on the Pmod pins. Checks LEDs, every display view,
// the real 7-segment pin patterns, and each fault switch.
//
// Fake card: block 0 = MBR (FAT32 partition at 2048), block 2048 = boot record
// (512 B/sector, 8 sectors/cluster, 32 reserved, 2 FATs x 7600, root cluster 2).
// Blocks at or past 0x01000000 are rejected with R1 = 0x40, like a real card.

module basys3_top_tb;

    reg clk = 0;
    always #5 clk = ~clk;

    reg  [15:0] sw = 0;
    reg         btnC = 0, btnU = 0, btnD = 0;
    wire [15:0] led;
    wire [6:0]  seg;
    wire        dp;
    wire [3:0]  an;
    wire        sd_cs_n, sd_mosi, sd_sclk;
    wire        sd_miso;

    basys3_top dut (
        .clk(clk), .btnC(btnC), .btnU(btnU), .btnD(btnD), .sw(sw), .led(led),
        .seg(seg), .dp(dp), .an(an),
        .sd_cs_n(sd_cs_n), .sd_mosi(sd_mosi), .sd_miso(sd_miso), .sd_sclk(sd_sclk)
    );

    // ==================================================================
    // Disk contents
    // ==================================================================
    reg [7:0] mbr [0:511];
    reg [7:0] vbr [0:511];
    integer k;
    initial begin
        for (k = 0; k < 512; k = k + 1) begin mbr[k] = 8'h00; vbr[k] = 8'h00; end
        mbr[450] = 8'h0C;
        mbr[454] = 8'h00; mbr[455] = 8'h08; mbr[456] = 8'h00; mbr[457] = 8'h00;   // 2048
        mbr[510] = 8'h55; mbr[511] = 8'hAA;
        vbr[11] = 8'h00; vbr[12] = 8'h02; vbr[13] = 8'd8;
        vbr[14] = 8'd32; vbr[16] = 8'd2;
        vbr[36] = 8'hB0; vbr[37] = 8'h1D;   // 7600
        vbr[44] = 8'd2;
        vbr[510] = 8'h55; vbr[511] = 8'hAA;
    end
    function [7:0] disk_byte(input [31:0] blk, input integer i);
        begin
            if (blk == 0)          disk_byte = mbr[i];
            else if (blk == 2048)  disk_byte = vbr[i];
            else                   disk_byte = 8'h00;
        end
    endfunction

    // ==================================================================
    // Fake SD card (SPI slave)
    // ==================================================================
    reg [7:0] q [0:2047];
    integer   q_head = 0, q_tail = 0;
    task q_push(input [7:0] b); begin q[q_tail % 2048] = b; q_tail = q_tail + 1; end endtask

    reg [7:0] card_tx = 8'hFF, rx_shift = 8'h00;
    integer   rise_bits = 0, fall_bits = 0;
    assign sd_miso = sd_cs_n ? 1'b1 : card_tx[7];

    reg [7:0] cmd_buf [0:5];
    integer   cmd_pos = 0, acmd41_seen = 0;
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
    always @(posedge sd_sclk) if (!sd_cs_n) begin
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

    // SCLK period while CS is low (data phase)
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

    // press btnC (reset + apply fault switches), then wait for mounted or any error
    integer cycles;
    task press_reset_and_wait;
        begin
            card_reset;
            @(negedge clk); btnC = 1;
            repeat (20) @(negedge clk);
            btnC = 0;
            repeat (10) @(negedge clk);
            cycles = 0;
            while (!led[1] && !led[2] && !led[3] && cycles < 4_000_000) begin @(negedge clk); cycles = cycles + 1; end
            repeat (200) @(negedge clk);
            $display("  LEDs = %b_%b_%b_%b   (faults | mount err | SD err | M S mnt rdy)",
                     led[15:12], led[11:8], led[7:4], led[3:0]);
        end
    endtask

    // put one view switch up and read the value going to the display
    task view(input integer n, input [15:0] expect, input [8*40-1:0] name);
        begin
            sw[11:0] = 12'd0;
            if (n >= 0) sw[n] = 1'b1;
            repeat (5) @(negedge clk);
            $display("  view SW%0d %0s = %04h", n, name, dut.disp);
            check(dut.disp == expect, name);
            sw[11:0] = 12'd0;
        end
    endtask

    // decode what the 7-segment pins are showing on one digit (0 = rightmost)
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
        check(led[1:0] == 2'b11 && led[3:2] == 2'b00, "LD0 + LD1 on, no errors");
        check(led[15:12] == 4'b0000,                  "no faults applied");
        check(sclk_period > 99.0 && sclk_period < 101.0, "SPI at 10 MHz (100 ns period)");
        $display("  SCLK period = %0.1f ns", sclk_period);
        view(-1, 16'h0000, "error codes");
        view(0,  16'h0101, "CMD0 tries/R1");
        view(1,  16'h01AA, "CMD8 echo");
        view(2,  16'h0300, "ACMD41 tries/R1");
        view(3,  16'hC0FF, "CMD58 OCR upper");
        view(4,  16'h00FE, "CMD17 R1/token");
        view(5,  16'h0003, "bytes before token");
        view(6,  16'h0800, "part_lba");
        view(7,  16'h0820, "fat_lba");
        view(8,  16'h4380, "data_lba");
        view(9,  16'h0008, "sectors/cluster");
        view(10, 16'h0002, "root cluster");
        view(11, 16'h0010, "SPI speed 10 MHz");

        // lowest switch wins when several are up
        sw[1] = 1; sw[4] = 1; repeat (5) @(negedge clk);
        check(dut.disp == 16'h01AA, "two view switches up: lowest (SW1) wins");
        sw[11:0] = 0;

        // the real pins: scan the 7-segment display
        sw[1] = 1;
        read_display;
        $display("  7-segment pins show %04h", shown);
        check(shown == 16'h01AA, "7-segment pins actually show 01AA");
        check(dp == 1'b1,        "decimal point off (value fits in 16 bits)");
        sw[1] = 0;

        // ---------------- SW15: ignore MISO ----------------
        $display("");
        $display("[FAULT SW15] FPGA ignores MISO");
        sw[15] = 1;
        press_reset_and_wait;
        check(led[2] && led[7:4] == 4'd1, "SD error 1 (CMD0 never answered)");
        check(led[15:12] == 4'b1000,      "LD15 shows the applied fault");
        view(0, 16'h14FF, "CMD0: 20 tries, no reply");
        sw[15] = 0;

        // ---------------- SW14: 25 MHz ----------------
        $display("");
        $display("[FAULT SW14] SPI at 25 MHz (the fake card copes; the real one fails with error 7)");
        sw[14] = 1;
        press_reset_and_wait;
        $display("  SCLK period = %0.1f ns", sclk_period);
        check(sclk_period > 39.0 && sclk_period < 41.0, "SPI really at 25 MHz (40 ns period)");
        check(led[14],                                  "LD14 shows the applied fault");
        view(11, 16'h0025, "SPI speed 25 MHz");
        sw[14] = 0;

        // ---------------- SW13: block 1 as MBR ----------------
        $display("");
        $display("[FAULT SW13] MBR read from block 1");
        sw[13] = 1;
        press_reset_and_wait;
        check(led[3] && led[11:8] == 4'd2 && !led[2], "mount error 2, SD side fine");
        check(last_read == 32'd1,                     "card was asked for block 1");
        view(-1, 16'h0002, "error codes 00 / 02");
        sw[13] = 0;

        // ---------------- SW12: past the end ----------------
        $display("");
        $display("[FAULT SW12] MBR read from block FFFFFFFF");
        sw[12] = 1;
        press_reset_and_wait;
        check(led[2] && led[7:4] == 4'd5, "SD error 5 (card rejected the address)");
        view(4,  16'h4000, "CMD17: card replied 40");
        view(-1, 16'h0501, "error codes 05 / 01");
        sw[12] = 0;

        // ---------------- Back to normal ----------------
        $display("");
        $display("[NORMAL AGAIN] all switches down, press btnC");
        press_reset_and_wait;
        check(led[1:0] == 2'b11 && led[15:2] == 14'd0, "mounted, no errors, no faults");

        $display("");
        $display("==============================================");
        $display(" RESULTS: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display(" ALL TESTS PASSED");
        else                 $display(" SOME TESTS FAILED - see lines marked <-----");
        $display("==============================================");
        $finish;
    end

    initial begin #300_000_000; $display("GLOBAL TIMEOUT"); $finish; end

endmodule
