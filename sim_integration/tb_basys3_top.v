`timescale 1ns / 1ps
// System testbench: basys3_top = SD_controller + sd_block_adapter + fat32_top
//
// Bit-level SPI SD card model (from SD_FAT32_top_tb.v) serving a FAT32 image
// built by make_fat32_image.py. Run from this folder with run.sh.
//
//   1. power-up -> card startup -> mount -> find VIDEO.BIN -> stream it
//      every streamed byte is compared with the real file, plus the
//      placeholder byte counter / XOR checksum, LEDs and 7-seg selection
//   2. btnU replay streams the whole file again
//   3. the card rejects one data block (R1 = 0x40) during a replay:
//      SD error -> adapter rd_err pulse -> reader_err = 1
//   4. btnC reset recovers everything

module tb_basys3_top;

    reg clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    // ---------------- DUT ----------------
    reg         btnC = 1'b0, btnU = 1'b0;
    reg  [15:0] sw = 16'h0000;
    wire [15:0] led;
    wire [6:0]  seg;
    wire        dp;
    wire [3:0]  an;
    wire        sclk, mosi, miso, cs_n;

    basys3_top #(
        .POWER_WAIT_CYCLES(1_000),
        .ACMD41_TIMEOUT(500_000),
        .TOKEN_TIMEOUT(20_000),
        .FAST_HALF(9'd2),           // 25 MHz to keep the sim short
        .DEBOUNCE_CYCLES(100)
    ) dut (
        .clk(clk), .btnC(btnC), .btnU(btnU), .sw(sw),
        .led(led), .seg(seg), .dp(dp), .an(an),
        .sd_cs_n(cs_n), .sd_mosi(mosi), .sd_miso(miso), .sd_sclk(sclk)
    );

    // ==================================================================
    // DISK IMAGE
    // ==================================================================
    localparam MAXB = 64;
    reg [31:0] lbas   [0:MAXB-1];
    reg [7:0]  blocks [0:MAXB*512-1];
    reg [7:0]  file   [0:65535];
    reg [31:0] expect [0:3];
    integer    nblocks;
    integer    k;

    initial begin
        for (k = 0; k < MAXB; k = k + 1) lbas[k] = 32'hFFFF_FFFF;
        $readmemh("lbas.hex",   lbas);
        $readmemh("blocks.hex", blocks);
        $readmemh("file.hex",   file);
        $readmemh("expect.hex", expect);
        nblocks = 0;
        while (nblocks < MAXB && lbas[nblocks] != 32'hFFFF_FFFF) nblocks = nblocks + 1;
    end

    function [7:0] disk_byte(input [31:0] blk, input integer i);
        integer j;
        begin
            disk_byte = 8'h00;
            for (j = 0; j < nblocks; j = j + 1)
                if (lbas[j] == blk) disk_byte = blocks[j*512 + i];
        end
    endfunction

    // ==================================================================
    // FAKE SD CARD (SPI slave, from SD_FAT32_top_tb)
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
    reg [31:0] fail_lba = 32'hFFFF_FFFF;    // CMD17 to this block is rejected

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
                    reads = reads + 1;
                    if (arg == fail_lba) q_push(8'h40);           // parameter error
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
    // STREAM CHECKER: every out_byte must match the file, in order
    // ==================================================================
    integer stream_pos = 0, mismatches = 0;
    always @(posedge clk) begin
        if (dut.sink_clear) stream_pos = 0;
        else if (dut.out_valid) begin
            if (stream_pos >= expect[2] || dut.out_byte !== file[stream_pos]) begin
                if (mismatches < 5)
                    $display("  MISMATCH at byte %0d: got %02h expected %02h",
                             stream_pos, dut.out_byte, file[stream_pos]);
                mismatches = mismatches + 1;
            end
            stream_pos = stream_pos + 1;
        end
    end

    // count rd_err pulses from the adapter
    integer rd_err_pulses = 0;
    always @(posedge clk) if (dut.rd_err) rd_err_pulses = rd_err_pulses + 1;

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

    task wait_done_or_error(input integer limit);
        integer c;
        begin
            c = 0;
            while (!dut.file_done && !dut.fat_error && !dut.sd_error && c < limit) begin
                @(negedge clk); c = c + 1;
            end
            repeat (5) @(negedge clk);
        end
    endtask

    task press_button_u;
        begin
            btnU = 1'b1; repeat (300) @(negedge clk);
            btnU = 1'b0; repeat (300) @(negedge clk);
        end
    endtask

    task check_stream(input [8*40-1:0] tag);
        begin
            $display("  [%0s] found=%0d done=%0d err=%0d loc_err=%0d rdr_err=%0d | count=%0d xor=%02h | mismatches=%0d reads=%0d",
                     tag, dut.found, dut.file_done, dut.fat_error, dut.locate_err, dut.reader_err,
                     dut.byte_count, dut.checksum, mismatches, reads);
            check(dut.found && dut.file_done && !dut.fat_error && !dut.sd_error, "file found and fully streamed, no errors");
            check(dut.byte_count == expect[2],          "byte counter == file size");
            check(dut.checksum == expect[3][7:0],       "XOR checksum matches the file");
            check(stream_pos == expect[2] && mismatches == 0, "every byte matches the file, in order");
        end
    endtask

    initial begin
        $display("");
        $display("==============================================");
        $display(" basys3_top system testbench (SD -> FAT32 -> stream)");
        $display("==============================================");
        @(negedge clk);
        $display("  image: %0d blocks, part_start=%0d first_cluster=%0d size=%0d xor=%02h",
                 nblocks, expect[0], expect[1], expect[2], expect[3][7:0]);

        // ---------------- 1. power-up run ----------------
        $display("");
        $display("[1] power-up: startup, mount, locate, stream");
        wait_done_or_error(3_000_000);
        check(dut.sd_init_done && !dut.sd_error,    "card startup finished");
        check(dut.part_start == expect[0],          "part_start correct");
        check(dut.first_cluster == expect[1],       "first_cluster correct (high word used)");
        check(dut.file_size == expect[2],           "file_size correct");
        check_stream("run 1");
        check(rd_err_pulses == 0,                   "no rd_err pulses");

        // LEDs: {hb, sd_err_code, sd_error, sd_init_done, reader_err, locate_err, file_done, fat_error, found}
        check(led[14:0] == 15'b0000_0_1_00_0000_1_0_1, "LEDs: found, file_done, card ready, no errors");

        // 7-segment selection (combinational disp/big)
        sw = 16'h0000; repeat (3) @(negedge clk);
        check(dut.disp == expect[0][15:0] && !dut.big,                     "SW0 down: part_start");
        sw = 16'h0001; repeat (3) @(negedge clk);
        check(dut.disp == expect[1][15:0] && dut.big == |expect[1][31:16], "SW0 up: first_cluster, DP = high bits");
        sw = 16'h0002; repeat (3) @(negedge clk);
        check(dut.disp == {8'h00, expect[3][7:0]},                          "SW1: checksum");
        sw = 16'h0004; repeat (3) @(negedge clk);
        check(dut.disp == expect[2][15:0],                                  "SW2: byte count");
        sw = 16'h0100; repeat (3) @(negedge clk);
        check(dut.disp == 16'h00FE,                                         "SW8: CMD17 R1/token = 00FE");
        sw = 16'h0000;

        // ---------------- 2. replay ----------------
        $display("");
        $display("[2] btnU replay");
        reads = 0;
        press_button_u;
        check(dut.byte_count < expect[2],           "replay restarted the stream");
        wait_done_or_error(3_000_000);
        check_stream("replay");
        check(reads == 11 + 5,                      "replay = 11 data + 5 FAT block reads");

        // ---------------- 3. SD read error during replay ----------------
        $display("");
        $display("[3] card rejects the 5th data block (first sector of cluster 10) during a replay");
        fail_lba = 32'd6080 + (32'd10 - 32'd2) * 2;     // DATA_START + (10-2)*SPC
        press_button_u;
        wait_done_or_error(3_000_000);
        repeat (20) @(negedge clk);
        $display("  sd_error=%0d sd_code=%0d reader_err=%0d fat_error=%0d count=%0d rd_err_pulses=%0d",
                 dut.sd_error, dut.sd_err_code, dut.reader_err, dut.fat_error, dut.byte_count, rd_err_pulses);
        check(dut.sd_error && dut.sd_err_code == 4'd5, "SD error 5 (CMD17 rejected)");
        check(rd_err_pulses == 1,                   "adapter turned the error level into ONE rd_err pulse");
        check(dut.fat_error && dut.reader_err == 2'd1, "reader_err = 1 (SD read error)");
        check(!dut.file_done && dut.byte_count == 4*512, "stream stopped after the 4 good sectors");
        check(led[1] && led[10] && led[8:7] == 2'd1, "LEDs show FAT32 error, SD error, reader_err");

        // ---------------- 4. btnC reset ----------------
        $display("");
        $display("[4] btnC reset recovers everything");
        fail_lba = 32'hFFFF_FFFF; acmd41_seen = 0; reads = 0;
        btnC = 1'b1; repeat (10) @(negedge clk); btnC = 1'b0;
        repeat (5) @(negedge clk);
        check(!dut.sd_error && !dut.found && dut.byte_count == 0, "reset cleared SD, FAT32 and the sink");
        wait_done_or_error(3_000_000);
        check_stream("after reset");
        check(reads == 6 + 16,                      "mount+locate 6 reads, stream 16 reads");

        $display("");
        $display("==============================================");
        $display(" RESULTS: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display(" ALL TESTS PASSED");
        else                 $display(" SOME TESTS FAILED - see lines marked <-----");
        $display("==============================================");
        $finish;
    end

    initial begin
        #200_000_000;
        $display("GLOBAL TIMEOUT - simulation hung");
        $finish;
    end

endmodule
