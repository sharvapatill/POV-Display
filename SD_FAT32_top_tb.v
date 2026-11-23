`timescale 1ns / 1ps
// Full-stack testbench: SD_FAT32_top (SD_controller + SD_cmd + SPI_driver + FAT32_mount)
//
// The fake card works at the SPI bit level (same model as SD_controller_tb),
// but CMD17 now serves real disk blocks:
//   block 0    = MBR  (FAT32 partition, type 0x0C, starts at block 2048)
//   block 2048 = VBR  (512 B/sector, 8 sectors/cluster, 32 reserved, 2 FATs x 7600, root = 2)
//   anything else = zeros

module SD_FAT32_top_tb;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;   // 100 MHz

    initial begin
        $dumpfile("sd_fat32_top.vcd");
        $dumpvars(1, SD_FAT32_top_tb);
        $dumpvars(1, SD_FAT32_top_tb.dut);
        $dumpvars(0, SD_FAT32_top_tb.dut.sd.state);
        $dumpvars(0, SD_FAT32_top_tb.dut.fat.state);
    end

    // ---------------- DUT ----------------
    reg         reinit = 0;
    reg         mount_start = 0;
    reg  [8:0]  fast_half = 9'd5;      // 10 MHz, the board's normal speed
    reg  [31:0] mbr_addr  = 32'd0;
    wire [15:0] dbg_cmd0, dbg_cmd8, dbg_acmd41, dbg_ocr, dbg_read, dbg_polls;
    wire        sd_init_done, sd_error, mounted, mount_busy, mount_error;
    wire [3:0]  sd_err_code, mount_err_code;
    wire [31:0] part_lba, fat_lba, data_lba, root_cluster;
    wire [7:0]  sectors_per_cluster;
    wire        sclk, mosi, miso, cs_n;

    SD_FAT32_top #(
        .POWER_WAIT_CYCLES(1_000),
        .ACMD41_TIMEOUT(500_000),
        .TOKEN_TIMEOUT(20_000)
    ) dut (
        .clk(clk), .rst(rst),
        .reinit(reinit), .mount_start(mount_start),
        .fast_half(fast_half), .mbr_addr(mbr_addr),
        .dbg_cmd0(dbg_cmd0), .dbg_cmd8(dbg_cmd8), .dbg_acmd41(dbg_acmd41),
        .dbg_ocr(dbg_ocr), .dbg_read(dbg_read), .dbg_polls(dbg_polls),
        .sd_init_done(sd_init_done), .sd_error(sd_error), .sd_err_code(sd_err_code),
        .mounted(mounted), .mount_busy(mount_busy),
        .mount_error(mount_error), .mount_err_code(mount_err_code),
        .part_lba(part_lba), .fat_lba(fat_lba), .data_lba(data_lba),
        .root_cluster(root_cluster), .sectors_per_cluster(sectors_per_cluster),
        .sd_sclk(sclk), .sd_mosi(mosi), .sd_miso(miso), .sd_cs_n(cs_n)
    );

    // ==================================================================
    // DISK CONTENTS
    // ==================================================================
    reg [7:0] mbr [0:511];
    reg [7:0] vbr [0:511];
    localparam [31:0] PART_START = 32'd2048;
    integer k;

    initial begin
        for (k = 0; k < 512; k = k + 1) begin mbr[k] = 8'h00; vbr[k] = 8'h00; end
        // MBR partition 1
        mbr[450] = 8'h0C;
        mbr[454] = PART_START[7:0];  mbr[455] = PART_START[15:8];
        mbr[456] = PART_START[23:16]; mbr[457] = PART_START[31:24];
        mbr[510] = 8'h55; mbr[511] = 8'hAA;
        // VBR
        vbr[0] = 8'hEB; vbr[1] = 8'h58; vbr[2] = 8'h90;
        vbr[11] = 8'h00; vbr[12] = 8'h02;          // 512 bytes/sector
        vbr[13] = 8'd8;                             // sectors/cluster
        vbr[14] = 8'd32; vbr[15] = 8'd0;            // reserved
        vbr[16] = 8'd2;                             // FATs
        vbr[36] = 8'hB0; vbr[37] = 8'h1D;           // 7600 = 0x1DB0
        vbr[44] = 8'd2;                             // root cluster
        vbr[510] = 8'h55; vbr[511] = 8'hAA;
    end

    function [7:0] disk_byte(input [31:0] blk, input integer i);
        begin
            if (blk == 0)               disk_byte = mbr[i];
            else if (blk == PART_START) disk_byte = vbr[i];
            else                        disk_byte = 8'h00;
        end
    endfunction

    // ==================================================================
    // FAKE SD CARD (SPI slave, from SD_controller_tb)
    // ==================================================================
    reg [7:0] q [0:2047];
    integer   q_head = 0, q_tail = 0;
    task q_push(input [7:0] b);
        begin q[q_tail % 2048] = b; q_tail = q_tail + 1; end
    endtask

    reg [7:0] card_tx = 8'hFF;
    reg [7:0] rx_shift = 8'h00;
    integer   rise_bits = 0, fall_bits = 0;
    assign miso = cs_n ? 1'b1 : card_tx[7];

    reg [7:0]  cmd_buf [0:5];
    integer    cmd_pos = 0;
    integer    acmd41_seen = 0;
    integer    reads = 0;
    reg [31:0] read_log [0:7];

    task handle_command;
        reg [5:0]  idx;
        reg [31:0] arg;
        integer    i;
        begin
            idx = cmd_buf[0][5:0];
            arg = {cmd_buf[1], cmd_buf[2], cmd_buf[3], cmd_buf[4]};
            q_push(8'hFF);
            case (idx)
                6'd0:  q_push(8'h01);
                6'd8:  begin q_push(8'h01); q_push(8'h00); q_push(8'h00);
                             q_push({4'h0, arg[11:8]}); q_push(arg[7:0]); end
                6'd55: q_push(8'h01);
                6'd41: begin acmd41_seen = acmd41_seen + 1;
                             q_push(acmd41_seen <= 2 ? 8'h01 : 8'h00); end
                6'd58: begin q_push(8'h00); q_push(8'hC0); q_push(8'hFF);
                             q_push(8'h80); q_push(8'h00); end
                6'd17: begin
                    if (reads < 8) read_log[reads] = arg;
                    reads = reads + 1;
                    if (arg >= 32'h0100_0000) q_push(8'h40);   // past the end of this 8 GB fake card: parameter error
                    else begin
                    q_push(8'h00);
                    q_push(8'hFF); q_push(8'hFF);
                    q_push(8'hFE);
                    for (i = 0; i < 512; i = i + 1) q_push(disk_byte(arg, i));
                    q_push(8'hAB); q_push(8'hCD);
                    end
                end
                default: q_push(8'h04);
            endcase
        end
    endtask

    always @(negedge cs_n) begin
        rise_bits = 0; fall_bits = 0;
        card_tx = (q_head < q_tail) ? q[q_head % 2048] : 8'hFF;
        if (q_head < q_tail) q_head = q_head + 1;
    end

    always @(posedge sclk) begin
        if (!cs_n) begin
            rx_shift  = {rx_shift[6:0], mosi};
            rise_bits = rise_bits + 1;
            if (rise_bits == 8) begin
                rise_bits = 0;
                if (cmd_pos == 0 && rx_shift[7:6] == 2'b01) begin
                    cmd_buf[0] = rx_shift; cmd_pos = 1;
                end else if (cmd_pos > 0) begin
                    cmd_buf[cmd_pos] = rx_shift;
                    cmd_pos = cmd_pos + 1;
                    if (cmd_pos == 6) begin cmd_pos = 0; handle_command; end
                end
            end
        end
    end

    always @(negedge sclk) begin
        if (!cs_n) begin
            fall_bits = fall_bits + 1;
            if (fall_bits == 8) begin
                fall_bits = 0;
                if (q_head < q_tail) begin card_tx = q[q_head % 2048]; q_head = q_head + 1; end
                else card_tx = 8'hFF;
            end else
                card_tx = {card_tx[6:0], 1'b1};
        end
    end

    // ==================================================================
    // TEST
    // ==================================================================
    integer pass_count = 0, fail_count = 0;
    task check(input cond, input [8*72-1:0] msg);
        begin
            if (cond) begin pass_count = pass_count + 1; $display("  PASS: %0s", msg); end
            else      begin fail_count = fail_count + 1; $display("  FAIL: %0s   <-----", msg); end
        end
    endtask

    integer cycles;
    initial begin
        $display("");
        $display("==============================================");
        $display(" SD_FAT32_top full-stack testbench");
        $display("==============================================");
        repeat (5) @(negedge clk);
        rst = 0;

        // No mount_start pulse: the top should auto-mount after card startup
        cycles = 0;
        while (!mounted && !mount_error && !sd_error && cycles < 3_000_000) begin
            @(negedge clk); cycles = cycles + 1;
        end
        repeat (3) @(negedge clk);

        $display("  sd_init_done=%0d sd_error=%0d sd_err=%0d | mounted=%0d mount_error=%0d mount_err=%0d",
                 sd_init_done, sd_error, sd_err_code, mounted, mount_error, mount_err_code);
        $display("  CMD17 reads=%0d (first %0d, second %0d)", reads, read_log[0], read_log[1]);
        $display("  part_lba=%0d fat_lba=%0d data_lba=%0d root_cluster=%0d spc=%0d",
                 part_lba, fat_lba, data_lba, root_cluster, sectors_per_cluster);

        check(sd_init_done && !sd_error,      "card startup finished");
        check(mounted && !mount_error,        "auto-mounted with no error");
        check(reads == 2,                     "exactly 2 CMD17 reads over SPI");
        check(read_log[0] == 0,               "first read = block 0 (MBR)");
        check(read_log[1] == 2048,            "second read = block 2048 (VBR)");
        check(part_lba == 2048,               "part_lba = 2048");
        check(fat_lba == 2080,                "fat_lba = 2080");
        check(data_lba == 17280,              "data_lba = 17280");
        check(root_cluster == 2,              "root_cluster = 2");
        check(sectors_per_cluster == 8,       "sectors_per_cluster = 8");

        // Manual remount through the real controller
        reads = 0;
        @(negedge clk); mount_start = 1; @(negedge clk); mount_start = 0;
        cycles = 0;
        while ((mount_busy || cycles < 2) && cycles < 1_000_000) begin @(negedge clk); cycles = cycles + 1; end
        repeat (3) @(negedge clk);
        check(mounted && !mount_error && reads == 2, "manual remount reads 2 blocks again");
        $display("  replies: CMD0=%04h CMD8=%04h ACMD41=%04h OCR=%04h READ=%04h polls=%0d",
                 dbg_cmd0, dbg_cmd8, dbg_acmd41, dbg_ocr, dbg_read, dbg_polls);
        check(dbg_cmd0 == 16'h0101 && dbg_cmd8 == 16'h01AA && dbg_read == 16'h00FE,
              "every command's reply captured correctly");

        // ---------------- Fault: block 1 treated as the MBR ----------------
        $display("");
        $display("[FAULT] MBR read from block 1 (no 55 AA there)");
        mbr_addr = 32'd1; reads = 0;
        @(negedge clk); mount_start = 1; @(negedge clk); mount_start = 0;
        cycles = 0;
        while (!mount_error && cycles < 1_000_000) begin @(negedge clk); cycles = cycles + 1; end
        repeat (3) @(negedge clk);
        $display("  mount_error=%0d code=%0d first read=%0d", mount_error, mount_err_code, read_log[0]);
        check(mount_error && mount_err_code == 4'd2 && read_log[0] == 1, "mount error 2 (bad MBR signature)");
        check(!sd_error, "SD side still healthy");

        // ---------------- Fault: read past the end of the card ----------------
        $display("");
        $display("[FAULT] MBR read from block FFFFFFFF (past the end of the card)");
        mbr_addr = 32'hFFFF_FFFF; reads = 0;
        @(negedge clk); mount_start = 1; @(negedge clk); mount_start = 0;
        cycles = 0;
        while (!sd_error && cycles < 1_000_000) begin @(negedge clk); cycles = cycles + 1; end
        repeat (10) @(negedge clk);
        $display("  sd_error=%0d sd_code=%0d mount_code=%0d read reply=%04h", sd_error, sd_err_code, mount_err_code, dbg_read);
        check(sd_error && sd_err_code == 4'd5, "SD error 5 (card rejected the address)");
        check(dbg_read[15:8] == 8'h40,         "card's reply 0x40 captured");
        check(mount_error && mount_err_code == 4'd1, "mount error 1 (SD reported an error)");

        // ---------------- Recovery ----------------
        $display("");
        $display("[RECOVERY] mbr_addr back to 0, reinit");
        mbr_addr = 32'd0; reads = 0; acmd41_seen = 0;
        @(negedge clk); reinit = 1; @(negedge clk); reinit = 0;
        cycles = 0;
        while (!mounted && cycles < 3_000_000) begin @(negedge clk); cycles = cycles + 1; end
        repeat (3) @(negedge clk);
        check(mounted && !mount_error && !sd_error && data_lba == 17280, "re-initialized and mounted again");

        $display("");
        $display("==============================================");
        $display(" RESULTS: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display(" ALL TESTS PASSED");
        else                 $display(" SOME TESTS FAILED - see lines marked <-----");
        $display("==============================================");
        $finish;
    end

    initial begin
        #100_000_000;
        $display("GLOBAL TIMEOUT - simulation hung");
        $finish;
    end

endmodule
