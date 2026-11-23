`timescale 1ns / 1ps
// Self-checking testbench for FAT32_mount
//
// Instead of the full SD stack (already tested on its own), this uses a fake
// "block device" that behaves like SD_controller's read port:
//   start_read -> busy -> a short wait -> 512 x (data_out + data_valid + byte_index)
//   -> block_done
// The fake disk holds two blocks we build by hand: the MBR (block 0) and the
// partition's boot record (VBR). Each test changes one field to hit one path.

module FAT32_mount_tb;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;   // 100 MHz

    initial begin
        $dumpfile("fat32_mount.vcd");
        $dumpvars(0, FAT32_mount_tb);
    end

    // ---------------- DUT ----------------
    reg         mount_start = 0;
    wire        mounted, busy, error;
    wire [3:0]  err_code;
    wire [31:0] part_lba, fat_lba, data_lba, root_cluster;
    wire [7:0]  sectors_per_cluster;

    reg         sd_init_done = 0;
    reg         sd_busy      = 0;
    reg         sd_error     = 0;
    wire        sd_start_read;
    wire [31:0] sd_block_addr;
    reg  [7:0]  sd_data       = 0;
    reg         sd_data_valid = 0;
    reg  [8:0]  sd_byte_index = 0;
    reg         sd_block_done = 0;

    FAT32_mount dut (
        .clk(clk), .rst(rst),
        .mount_start(mount_start), .mbr_addr(32'd0),
        .mounted(mounted), .busy(busy), .error(error), .err_code(err_code),
        .part_lba(part_lba), .fat_lba(fat_lba), .data_lba(data_lba),
        .root_cluster(root_cluster), .sectors_per_cluster(sectors_per_cluster),
        .sd_init_done(sd_init_done), .sd_busy(sd_busy), .sd_error(sd_error),
        .sd_start_read(sd_start_read), .sd_block_addr(sd_block_addr),
        .sd_data(sd_data), .sd_data_valid(sd_data_valid),
        .sd_byte_index(sd_byte_index), .sd_block_done(sd_block_done)
    );

    // ==================================================================
    // FAKE DISK
    // ==================================================================
    reg [7:0]  mbr [0:511];
    reg [7:0]  vbr [0:511];
    reg [31:0] vbr_at = 0;        // block number where the VBR lives
    reg        fail_reads = 0;    // fault: SD controller errors on a read

    integer    read_n = 0;        // how many blocks were requested
    reg [31:0] read_log [0:7];
    integer    bad_addr = 0;      // reads of blocks that are neither MBR nor VBR


    // little-endian writers
    task mbr_w32(input integer off, input [31:0] v);
        begin mbr[off] = v[7:0]; mbr[off+1] = v[15:8]; mbr[off+2] = v[23:16]; mbr[off+3] = v[31:24]; end
    endtask
    task vbr_w16(input integer off, input [15:0] v);
        begin vbr[off] = v[7:0]; vbr[off+1] = v[15:8]; end
    endtask
    task vbr_w32(input integer off, input [31:0] v);
        begin vbr[off] = v[7:0]; vbr[off+1] = v[15:8]; vbr[off+2] = v[23:16]; vbr[off+3] = v[31:24]; end
    endtask

    integer k;
    task build_mbr(input [7:0] ptype, input [31:0] start, input sig_ok);
        begin
            for (k = 0; k < 512; k = k + 1) mbr[k] = 8'h00;
            mbr[446] = 8'h00;              // not bootable
            mbr[450] = ptype;
            mbr_w32(454, start);
            mbr_w32(458, 32'd1_000_000);   // partition size (not used)
            mbr[510] = sig_ok ? 8'h55 : 8'h00;
            mbr[511] = sig_ok ? 8'hAA : 8'h00;
            vbr_at = start;
        end
    endtask

    task build_vbr(input [15:0] bps, input [7:0] spc, input [15:0] rsv,
                   input [7:0] nfats, input [31:0] fatsz, input [31:0] root, input sig_ok);
        begin
            for (k = 0; k < 512; k = k + 1) vbr[k] = 8'h00;
            vbr[0] = 8'hEB; vbr[1] = 8'h58; vbr[2] = 8'h90;   // jump instruction
            vbr_w16(11, bps);
            vbr[13] = spc;
            vbr_w16(14, rsv);
            vbr[16] = nfats;
            vbr_w32(36, fatsz);
            vbr_w32(44, root);
            // "FAT32   " label at 82 (not checked, just realistic)
            vbr[82] = "F"; vbr[83] = "A"; vbr[84] = "T"; vbr[85] = "3"; vbr[86] = "2";
            vbr[510] = sig_ok ? 8'h55 : 8'h00;
            vbr[511] = sig_ok ? 8'hAA : 8'h00;
        end
    endtask

    function [7:0] disk_byte(input [31:0] blk, input integer i);
        begin
            if (blk == 0)            disk_byte = mbr[i];
            else if (blk == vbr_at)  disk_byte = vbr[i];
            else                     disk_byte = 8'h00;
        end
    endfunction

    // Behaves like SD_controller: only accepts start_read when ready and idle
    reg [31:0] addr;
    integer    i;
    initial begin
        forever begin
            @(posedge clk);
            if (sd_start_read && sd_init_done && !sd_busy && !sd_error) begin
                addr = sd_block_addr;
                if (read_n < 8) read_log[read_n] = addr;
                read_n = read_n + 1;
                if (addr != 0 && addr != vbr_at) bad_addr = bad_addr + 1;
                sd_busy <= 1;
                repeat (20) @(posedge clk);              // command + waiting for 0xFE
                if (fail_reads) begin
                    sd_error <= 1;                       // like err_code 5/6/7
                    sd_busy  <= 0;
                end else begin
                    for (i = 0; i < 512; i = i + 1) begin
                        sd_data       <= disk_byte(addr, i);
                        sd_byte_index <= i;
                        sd_data_valid <= 1;
                        @(posedge clk);
                        sd_data_valid <= 0;
                        repeat (3) @(posedge clk);       // gap between bytes, like the real SPI
                    end
                    repeat (4) @(posedge clk);           // 2 CRC bytes
                    sd_block_done <= 1;
                    sd_busy       <= 0;
                    @(posedge clk);
                    sd_block_done <= 0;
                end
            end
        end
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

    integer cycles;
    reg     hung;
    task do_mount;
        begin
            read_n = 0; bad_addr = 0; hung = 0;
            @(negedge clk); mount_start = 1;
            @(negedge clk); mount_start = 0;
            cycles = 0;
            while (!mounted && !error && cycles < 100_000) begin
                @(negedge clk); cycles = cycles + 1;
            end
            if (!mounted && !error) hung = 1;
            repeat (3) @(negedge clk);
        end
    endtask

    task show;
        begin
            $display("  mounted=%0d error=%0d err_code=%0d | blocks read=%0d (first %0d, second %0d)",
                     mounted, error, err_code, read_n, read_log[0], read_log[1]);
            $display("  part_lba=%0d fat_lba=%0d data_lba=%0d root_cluster=%0d sectors/cluster=%0d",
                     part_lba, fat_lba, data_lba, root_cluster, sectors_per_cluster);
        end
    endtask

    // ==================================================================
    // TESTS
    // ==================================================================
    initial begin
        $display("");
        $display("==============================================");
        $display(" FAT32_mount testbench");
        $display("==============================================");

        repeat (5) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        // ---------------- TEST 1: waits for the card ----------------
        $display("");
        $display("[TEST 1] Mount requested before the SD card is ready");
        build_mbr(8'h0C, 32'd2048, 1);
        build_vbr(16'd512, 8'd8, 16'd32, 8'd2, 32'd7600, 32'd2, 1);
        read_n = 0;
        @(negedge clk); mount_start = 1; @(negedge clk); mount_start = 0;
        repeat (200) @(negedge clk);
        check(read_n == 0 && busy && !mounted, "no reads while sd_init_done is low");
        sd_init_done = 1;   // card finishes startup
        cycles = 0;
        while (!mounted && !error && cycles < 100_000) begin @(negedge clk); cycles = cycles + 1; end
        repeat (3) @(negedge clk);
        show;
        check(mounted && !error,                 "mounted once the card was ready");

        // ---------------- TEST 2: typical card ----------------
        $display("");
        $display("[TEST 2] Typical 32 GB card: start 2048, 32 reserved, 2 FATs x 7600, 8 sectors/cluster");
        do_mount;
        show;
        check(!hung,                             "finished");
        check(mounted && !error,                 "mounted, no error");
        check(read_n == 2,                       "read exactly 2 blocks");
        check(read_log[0] == 0,                  "first read = block 0 (MBR)");
        check(read_log[1] == 2048,               "second read = block 2048 (VBR)");
        check(part_lba == 2048,                  "part_lba = 2048");
        check(fat_lba == 2048 + 32,              "fat_lba = 2080");
        check(data_lba == 2080 + 2*7600,         "data_lba = 17280");
        check(root_cluster == 2,                 "root_cluster = 2");
        check(sectors_per_cluster == 8,          "sectors_per_cluster = 8");

        // ---------------- TEST 3: different layout, all bytes used ----------------
        $display("");
        $display("[TEST 3] Type 0x0B, big numbers in every byte (checks byte order)");
        build_mbr(8'h0B, 32'h0102_0304, 1);
        build_vbr(16'd512, 8'd64, 16'h0123, 8'd1, 32'h0004_5678, 32'h0A0B_0C0D, 1);
        do_mount;
        show;
        check(mounted && !error,                         "mounted, no error");
        check(read_log[1] == 32'h0102_0304,              "VBR read from little-endian start block");
        check(part_lba == 32'h0102_0304,                 "part_lba = 0x01020304");
        check(fat_lba == 32'h0102_0304 + 32'h0123,       "fat_lba = part + 0x123");
        check(data_lba == 32'h0102_0304 + 32'h0123 + 32'h0004_5678, "data_lba = fat + 1 x fat_size");
        check(root_cluster == 32'h0A0B_0C0D,             "root_cluster = 0x0A0B0C0D");
        check(sectors_per_cluster == 64,                 "sectors_per_cluster = 64");

        // ---------------- TEST 4: no MBR signature ----------------
        $display("");
        $display("[TEST 4] Block 0 has no 55 AA signature");
        build_mbr(8'h0C, 32'd2048, 0);
        build_vbr(16'd512, 8'd8, 16'd32, 8'd2, 32'd7600, 32'd2, 1);
        do_mount;
        show;
        check(error && err_code == 4'd2 && !mounted, "error code 2 (bad MBR signature)");
        check(read_n == 1,                       "stopped after 1 block");

        // ---------------- TEST 5: not FAT32 ----------------
        $display("");
        $display("[TEST 5] Partition is NTFS/exFAT (type 0x07)");
        build_mbr(8'h07, 32'd2048, 1);
        do_mount;
        show;
        check(error && err_code == 4'd3, "error code 3 (not FAT32)");

        // ---------------- TEST 6: bad VBR signature ----------------
        $display("");
        $display("[TEST 6] Boot record has no 55 AA signature");
        build_mbr(8'h0C, 32'd2048, 1);
        build_vbr(16'd512, 8'd8, 16'd32, 8'd2, 32'd7600, 32'd2, 0);
        do_mount;
        show;
        check(error && err_code == 4'd4, "error code 4 (bad VBR signature)");
        check(read_n == 2,               "both blocks were read");

        // ---------------- TEST 7: unsupported sector size ----------------
        $display("");
        $display("[TEST 7] 4096-byte sectors");
        build_vbr(16'd4096, 8'd8, 16'd32, 8'd2, 32'd7600, 32'd2, 1);
        do_mount;
        show;
        check(error && err_code == 4'd5, "error code 5 (unsupported format)");

        // ---------------- TEST 8: SD read fails ----------------
        $display("");
        $display("[TEST 8] SD controller errors during the read");
        build_vbr(16'd512, 8'd8, 16'd32, 8'd2, 32'd7600, 32'd2, 1);
        fail_reads = 1;
        do_mount;
        show;
        check(!hung,                     "did not hang");
        check(error && err_code == 4'd1, "error code 1 (SD error)");

        // ---------------- TEST 9: recovery ----------------
        $display("");
        $display("[TEST 9] Remount after errors (SD controller reinitialized)");
        fail_reads = 0;
        @(negedge clk); sd_error = 0;   // what reinit does to the real controller
        do_mount;
        show;
        check(mounted && !error && err_code == 0, "mounted again, error cleared");
        check(data_lba == 17280,                  "same layout as test 2");

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
        #50_000_000;
        $display("GLOBAL TIMEOUT - simulation hung");
        $finish;
    end

endmodule