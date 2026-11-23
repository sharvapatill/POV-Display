`timescale 1ns / 1ps
`default_nettype none
// Basys 3 board top: SD card -> FAT32 -> VIDEO.BIN byte stream (POV display)
//
//   basys3_top
//   ├── SD_controller     (owns the SPI pins, runs card startup, reads blocks)
//   │   ├── SPI_driver
//   │   └── SD_cmd
//   ├── sd_block_adapter  (SD_controller <-> fat32_top handshake)
//   └── fat32_top         (mount, find VIDEO.BIN, stream it)
//       ├── fat32_locate
//       └── fat32_reader
//
//   btnC = reset everything (card startup + mount + stream rerun automatically)
//   btnU = replay the file (only once it has finished or failed)
//   SW15 = slow SPI (2.5 MHz) when up while btnC is pressed
//   Pmod JA = SD card (JA1 CS, JA2 MOSI, JA3 MISO, JA4 SCK)
//
//   The file bytes go to a placeholder sink for now: a byte counter and an
//   XOR checksum. Compare them with the real file on a PC.
//
//   VIEW SWITCHES - 7-segment display, 4 hex digits.
//     SW4-SW9 (lowest up wins) SD debug:
//       SW4 CMD0 attempts/R1 (0101)   SW5 CMD8 echo (01AA)   SW6 ACMD41 (xx00)
//       SW7 OCR upper 16 (Cxxx)       SW8 CMD17 R1/token (00FE)
//       SW9 bytes waited before the token
//     else SW10-SW13 (lowest up wins) pattern checker, see make_pattern.py:
//       SW10 bad byte count   SW11 first bad offset (low 16)
//       SW12 first bad offset (high 16)   SW13 {expected, received}
//     else SW3  file_size (low 16 bits)
//     else SW2  bytes received (low 16 bits)
//     else SW1  XOR checksum of the bytes received (00xx)
//     else SW0  down = part_start (low 16 bits), up = first_cluster (low 16 bits)
//     Decimal point on the right digit = the value has more than 16 bits
//
//   LEDs:
//     LD0 found          LD1 FAT32 error     LD2 file_done
//     LD6-LD3 locate_err LD8-LD7 reader_err
//     LD9 card ready     LD10 SD error       LD14-LD11 SD error code
//     LD15 heartbeat (blinks = bitstream is running)

module basys3_top #(
    // Defaults are the real-board values; the testbench shrinks them.
    parameter POWER_WAIT_CYCLES = 100_000,      // 1 ms   @ 100 MHz
    parameter ACMD41_TIMEOUT    = 100_000_000,  // 1 s    @ 100 MHz
    parameter TOKEN_TIMEOUT     = 10_000_000,   // 100 ms @ 100 MHz
    parameter [8:0] FAST_HALF   = 9'd5,         // SPI after startup: 100 MHz / (2*5) = 10 MHz
    parameter DEBOUNCE_CYCLES   = 1_000_000     // 10 ms  @ 100 MHz
)(
    input  wire        clk,        // 100 MHz, W5
    input  wire        btnC,
    input  wire        btnU,
    input  wire [15:0] sw,
    output wire [15:0] led,
    output reg  [6:0]  seg,        // a..g, active low
    output reg         dp,         // decimal point, active low
    output reg  [3:0]  an,         // digit enables, active low

    output wire        sd_cs_n,
    output wire        sd_mosi,
    input  wire        sd_miso,
    output wire        sd_sclk
);

    // ---------------- Reset synchronizer ----------------
    // Asserts for 2 cycles at power-up, then follows btnC (synchronized).
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sync = 2'b11;
    always @(posedge clk) rst_sync <= {rst_sync[0], btnC};
    wire rst = rst_sync[1];

    // ---------------- Replay button: synchronize, debounce, rising edge ----------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] up_sync = 2'b00;
    reg        up_stable = 1'b0, up_stable_d = 1'b0;
    reg [19:0] up_cnt = 20'd0;
    always @(posedge clk) begin
        up_sync     <= {up_sync[0], btnU};
        up_stable_d <= up_stable;
        if (up_sync[1] == up_stable)
            up_cnt <= 20'd0;
        else if (up_cnt == DEBOUNCE_CYCLES - 1) begin
            up_cnt    <= 20'd0;
            up_stable <= up_sync[1];
        end else
            up_cnt <= up_cnt + 20'd1;
    end
    wire replay_pulse = up_stable & ~up_stable_d;

    // ---------------- SPI speed: SW15 sampled while reset is held ----------------
    // SW15 up at reset = 2.5 MHz (signal-integrity test), down = FAST_HALF.
    reg slow_spi = 1'b0;
    always @(posedge clk) if (rst) slow_spi <= sw[15];
    wire [8:0] fast_half = slow_spi ? 9'd20 : FAST_HALF;

    // ---------------- SD controller ----------------
    wire        sd_init_done, sd_busy, sd_error;
    wire [3:0]  sd_err_code;
    wire        sd_start_read;
    wire [31:0] sd_block_addr;
    wire [7:0]  sd_data;
    wire        sd_data_valid, sd_block_done;
    wire [15:0] dbg_cmd0, dbg_cmd8, dbg_acmd41, dbg_ocr, dbg_read, dbg_polls;

    SD_controller #(
        .POWER_WAIT_CYCLES(POWER_WAIT_CYCLES),
        .ACMD41_TIMEOUT(ACMD41_TIMEOUT),
        .TOKEN_TIMEOUT(TOKEN_TIMEOUT)
    ) sd (
        .clk(clk), .rst(rst),
        .reinit(1'b0),                      // btnC reset reruns startup instead
        .start_read(sd_start_read),
        .block_addr(sd_block_addr),
        .fast_half(fast_half),
        .init_done(sd_init_done),
        .busy(sd_busy),
        .error(sd_error),
        .err_code(sd_err_code),
        .data_out(sd_data),
        .data_valid(sd_data_valid),
        .byte_index(),                      // fat32_* count bytes themselves
        .block_done(sd_block_done),
        .dbg_cmd0(dbg_cmd0), .dbg_cmd8(dbg_cmd8), .dbg_acmd41(dbg_acmd41),
        .dbg_ocr(dbg_ocr), .dbg_read(dbg_read), .dbg_polls(dbg_polls),
        .sclk(sd_sclk), .mosi(sd_mosi), .miso(sd_miso), .cs_n(sd_cs_n)
    );

    // ---------------- Adapter ----------------
    wire        rd_start, rd_valid, rd_done, rd_err;
    wire [31:0] rd_lba;
    wire [7:0]  rd_byte;

    sd_block_adapter adapter (
        .clk(clk), .rst(rst),
        .rd_start(rd_start), .rd_lba(rd_lba),
        .rd_valid(rd_valid), .rd_byte(rd_byte),
        .rd_done(rd_done),   .rd_err(rd_err),
        .start_read(sd_start_read), .block_addr(sd_block_addr),
        .init_done(sd_init_done), .busy(sd_busy), .error(sd_error),
        .data_out(sd_data), .data_valid(sd_data_valid), .block_done(sd_block_done)
    );

    // ---------------- Start FAT32 when card startup finishes ----------------
    reg init_done_d;
    always @(posedge clk) init_done_d <= rst ? 1'b0 : sd_init_done;
    wire fat_start = sd_init_done & ~init_done_d;   // rising edge -> 1-cycle pulse

    // ---------------- FAT32 ----------------
    wire        out_valid, file_done, found, fat_error;
    wire [7:0]  out_byte;
    wire [3:0]  locate_err;
    wire [1:0]  reader_err;
    wire [31:0] first_cluster, file_size, part_start;

    fat32_top fat (
        .clk(clk), .rst(rst),
        .start(fat_start),
        .replay(replay_pulse),
        .ok_to_read(1'b1),                  // no frame buffer yet: stream flat out
        .sd_rd_start(rd_start), .sd_rd_lba(rd_lba),
        .sd_rd_valid(rd_valid), .sd_rd_byte(rd_byte),
        .sd_rd_done(rd_done),   .sd_rd_err(rd_err),
        .out_valid(out_valid), .out_byte(out_byte),
        .sector_done(),
        .file_done(file_done),
        .found(found), .error(fat_error),
        .locate_err(locate_err), .reader_err(reader_err),
        .first_cluster(first_cluster), .file_size(file_size),
        .bytes_sent(),
        .part_start(part_start)
    );

    // ---------------- Placeholder sink: byte count + XOR checksum ----------------
    // Cleared at each fresh start, and on a replay the reader will accept
    // (fat32_reader only restarts from DONE or ERR).
    reg [31:0] byte_count;
    reg [7:0]  checksum;
    wire sink_clear = fat_start | (replay_pulse & found & (file_done | fat_error));
    always @(posedge clk) begin
        if (rst || sink_clear) begin
            byte_count <= 32'd0;
            checksum   <= 8'h00;
        end else if (out_valid) begin
            byte_count <= byte_count + 32'd1;
            checksum   <= checksum ^ out_byte;
        end
    end

    // ---------------- Pattern checker (debug) ----------------
    // For a test VIDEO.BIN where byte i = i[7:0] ^ i[15:8] ^ i[23:16]
    // (test_files/make_pattern.py). Reports how many bytes differ and the
    // first one. Meaningless for any other file.
    wire [7:0] pat_expect = byte_count[7:0] ^ byte_count[15:8] ^ byte_count[23:16];
    reg [15:0] bad_count;
    reg [31:0] first_bad_off;
    reg [7:0]  first_bad_exp, first_bad_got;
    always @(posedge clk) begin
        if (rst || sink_clear) begin
            bad_count     <= 16'd0;
            first_bad_off <= 32'd0;
            first_bad_exp <= 8'h00;
            first_bad_got <= 8'h00;
        end else if (out_valid && out_byte != pat_expect) begin
            if (bad_count == 16'd0) begin
                first_bad_off <= byte_count;
                first_bad_exp <= pat_expect;
                first_bad_got <= out_byte;
            end
            if (bad_count != 16'hFFFF) bad_count <= bad_count + 16'd1;
        end
    end

    // ---------------- LEDs ----------------
    reg [25:0] heartbeat = 26'd0;
    always @(posedge clk) heartbeat <= heartbeat + 26'd1;

    assign led = {heartbeat[25], sd_err_code, sd_error, sd_init_done,
                  reader_err, locate_err, file_done, fat_error, found};

    // ---------------- What the 7-segment display shows ----------------
    (* ASYNC_REG = "TRUE" *) reg [13:0] sw_meta = 14'd0, sw_s = 14'd0;
    always @(posedge clk) begin
        sw_meta <= sw[13:0];
        sw_s    <= sw_meta;
    end

    reg [15:0] disp;      // 4 hex digits
    reg        big;       // value is wider than 16 bits -> light the decimal point
    always @(*) begin
        big = 1'b0;
        if      (sw_s[4]) disp = dbg_cmd0;
        else if (sw_s[5]) disp = dbg_cmd8;
        else if (sw_s[6]) disp = dbg_acmd41;
        else if (sw_s[7]) disp = dbg_ocr;
        else if (sw_s[8]) disp = dbg_read;
        else if (sw_s[9]) disp = dbg_polls;
        else if (sw_s[10]) disp = bad_count;
        else if (sw_s[11]) begin disp = first_bad_off[15:0]; big = |first_bad_off[31:16]; end
        else if (sw_s[12]) disp = first_bad_off[31:16];
        else if (sw_s[13]) disp = {first_bad_exp, first_bad_got};
        else if (sw_s[3]) begin disp = file_size[15:0];     big = |file_size[31:16];     end
        else if (sw_s[2]) begin disp = byte_count[15:0];    big = |byte_count[31:16];    end
        else if (sw_s[1]) disp = {8'h00, checksum};
        else if (sw_s[0]) begin disp = first_cluster[15:0]; big = |first_cluster[31:16]; end
        else            begin disp = part_start[15:0];    big = |part_start[31:16];    end
    end

    // ---------------- 7-segment driver ----------------
    // One digit lit at a time, each for about 1.3 ms -> looks steady to the eye.
    reg [18:0] refresh = 0;
    always @(posedge clk) refresh <= refresh + 19'd1;
    wire [1:0] digit = refresh[18:17];

    reg [3:0] nibble;
    always @(*) begin
        case (digit)
            2'd0: nibble = disp[3:0];     // rightmost
            2'd1: nibble = disp[7:4];
            2'd2: nibble = disp[11:8];
            default: nibble = disp[15:12]; // leftmost
        endcase
    end

    always @(posedge clk) begin
        an <= ~(4'b0001 << digit);
        dp <= ~(big && digit == 2'd0);
        case (nibble)               //  gfedcba
            4'h0: seg <= 7'b1000000;
            4'h1: seg <= 7'b1111001;
            4'h2: seg <= 7'b0100100;
            4'h3: seg <= 7'b0110000;
            4'h4: seg <= 7'b0011001;
            4'h5: seg <= 7'b0010010;
            4'h6: seg <= 7'b0000010;
            4'h7: seg <= 7'b1111000;
            4'h8: seg <= 7'b0000000;
            4'h9: seg <= 7'b0010000;
            4'hA: seg <= 7'b0001000;
            4'hB: seg <= 7'b0000011;
            4'hC: seg <= 7'b1000110;
            4'hD: seg <= 7'b0100001;
            4'hE: seg <= 7'b0000110;
            default: seg <= 7'b0001110;   // F
        endcase
    end

endmodule
`default_nettype wire
