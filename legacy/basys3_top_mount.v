`timescale 1ns / 1ps
// Basys 3 board top: SD card + FAT32 mount with a switch-driven test panel
//
//   btnC = reset (also applies the fault switches)   btnU = re-init card   btnD = remount
//   Pmod JA = SD card (JA1 CS, JA2 MOSI, JA3 MISO, JA4 SCK)
//
//   FAULT SWITCHES - set, then press btnC. All down = normal run at 10 MHz.
//     SW15  FPGA ignores MISO (like no card)        -> SD error 1
//     SW14  SPI at 25 MHz instead of 10 MHz         -> SD error 7
//     SW13  read the MBR from block 1               -> mount error 2
//     SW12  read the MBR from a block past the end  -> SD error 5 (or 7, depends on the card)
//
//   VIEW SWITCHES - 7-segment display, 4 hex digits. Lowest switch that is up wins.
//     none  SD error code (left pair) / mount error code (right pair)   good: 0000
//     SW0   CMD0:   attempts / R1                good: 0101
//     SW1   CMD8:   voltage echo                 good: 01AA
//     SW2   ACMD41: attempts / final R1          good: xx00
//     SW3   CMD58:  OCR upper 16 bits            good: Cxxx
//     SW4   CMD17:  R1 / token                   good: 00FE
//     SW5   bytes waited before the token
//     SW6   part_lba     SW7  fat_lba     SW8  data_lba
//     SW9   sectors per cluster              SW10 root cluster
//     SW11  SPI speed in MHz (10 or 25)
//     Decimal point on the right digit = the value has more than 16 bits (upper half not shown)
//
//   LEDs:
//     LD0 card ready   LD1 mounted   LD2 SD error   LD3 mount error
//     LD7-LD4  SD error code     LD11-LD8 mount error code
//     LD15-LD12 the fault switches that were applied at the last reset

module basys3_top (
    input  wire        clk,        // 100 MHz, W5
    input  wire        btnC,
    input  wire        btnU,
    input  wire        btnD,
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
    reg [1:0] rst_sync = 2'b11;
    always @(posedge clk) rst_sync <= {rst_sync[0], btnC};
    wire rst = rst_sync[1];

    // ---------------- Fault switches: captured while reset is held ----------------
    reg [3:0] fault = 4'b0000;   // {no_miso, too_fast, mbr_block1, mbr_past_end}
    always @(posedge clk) if (rst) fault <= sw[15:12];

    wire f_no_miso  = fault[3];
    wire f_fast     = fault[2];
    wire f_block1   = fault[1];
    wire f_past_end = fault[0];

    wire [8:0]  fast_half = f_fast ? 9'd2 : 9'd5;              // 25 MHz : 10 MHz
    wire [31:0] mbr_addr  = f_past_end ? 32'hFFFF_FFFF :
                            f_block1   ? 32'd1 : 32'd0;
    wire        miso_in   = f_no_miso ? 1'b1 : sd_miso;        // idle-high, like an empty socket

    // ---------------- Button synchronizer + rising edge ----------------
    reg [2:0] up_s = 0, dn_s = 0;
    always @(posedge clk) begin
        up_s <= {up_s[1:0], btnU};
        dn_s <= {dn_s[1:0], btnD};
    end
    wire reinit_pulse  = up_s[1] & ~up_s[2];
    wire remount_pulse = dn_s[1] & ~dn_s[2];

    // ---------------- SD + FAT32 stack ----------------
    wire        sd_init_done, sd_error, mounted, mount_busy, mount_error;
    wire [3:0]  sd_err_code, mount_err_code;
    wire [31:0] part_lba, fat_lba, data_lba, root_cluster;
    wire [7:0]  sectors_per_cluster;
    wire [15:0] dbg_cmd0, dbg_cmd8, dbg_acmd41, dbg_ocr, dbg_read, dbg_polls;

    SD_FAT32_top stack (
        .clk(clk), .rst(rst),
        .reinit(reinit_pulse), .mount_start(remount_pulse),
        .fast_half(fast_half), .mbr_addr(mbr_addr),
        .sd_init_done(sd_init_done), .sd_error(sd_error), .sd_err_code(sd_err_code),
        .mounted(mounted), .mount_busy(mount_busy),
        .mount_error(mount_error), .mount_err_code(mount_err_code),
        .part_lba(part_lba), .fat_lba(fat_lba), .data_lba(data_lba),
        .root_cluster(root_cluster), .sectors_per_cluster(sectors_per_cluster),
        .dbg_cmd0(dbg_cmd0), .dbg_cmd8(dbg_cmd8), .dbg_acmd41(dbg_acmd41),
        .dbg_ocr(dbg_ocr), .dbg_read(dbg_read), .dbg_polls(dbg_polls),
        .sd_sclk(sd_sclk), .sd_mosi(sd_mosi), .sd_miso(miso_in), .sd_cs_n(sd_cs_n)
    );

    // ---------------- LEDs ----------------
    assign led = {fault, mount_err_code, sd_err_code,
                  mount_error, sd_error, mounted, sd_init_done};

    // ---------------- What the 7-segment display shows ----------------
    reg [15:0] disp;      // 4 hex digits
    reg        big;       // value is wider than 16 bits -> light the decimal point
    always @(*) begin
        big = 1'b0;
        if      (sw[0])  disp = dbg_cmd0;
        else if (sw[1])  disp = dbg_cmd8;
        else if (sw[2])  disp = dbg_acmd41;
        else if (sw[3])  disp = dbg_ocr;
        else if (sw[4])  disp = dbg_read;
        else if (sw[5])  disp = dbg_polls;
        else if (sw[6])  begin disp = part_lba[15:0];     big = |part_lba[31:16];     end
        else if (sw[7])  begin disp = fat_lba[15:0];      big = |fat_lba[31:16];      end
        else if (sw[8])  begin disp = data_lba[15:0];     big = |data_lba[31:16];     end
        else if (sw[9])  disp = {8'h00, sectors_per_cluster};
        else if (sw[10]) begin disp = root_cluster[15:0]; big = |root_cluster[31:16]; end
        else if (sw[11]) disp = f_fast ? 16'h0025 : 16'h0010;   // MHz, written as decimal digits
        else             disp = {4'h0, sd_err_code, 4'h0, mount_err_code};
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
