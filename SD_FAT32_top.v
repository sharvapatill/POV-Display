`timescale 1ns / 1ps
// SD card + FAT32 mount, wired together
//
//   SD_FAT32_top
//   ├── SD_controller   (owns the SPI pins, runs card startup, reads blocks)
//   │   ├── SD_cmd
//   │   └── SPI_driver
//   └── FAT32_mount     (asks the controller for blocks 0 and the VBR)
//
// Mounts automatically once the card finishes startup (and again after a
// reinit). mount_start still works for a manual remount.

module SD_FAT32_top #(
    parameter POWER_WAIT_CYCLES = 100_000,      // 1 ms   @ 100 MHz
    parameter ACMD41_TIMEOUT    = 100_000_000,  // 1 s    @ 100 MHz
    parameter TOKEN_TIMEOUT     = 10_000_000,   // 100 ms @ 100 MHz
    parameter SLOW_HALF         = 250
)(
    input  wire        clk,
    input  wire        rst,

    // ---------------- Control ----------------
    input  wire        reinit,           // rerun card startup (after an error)
    input  wire        mount_start,      // optional manual remount
    input  wire [8:0]  fast_half,        // SPI speed after startup = 100 MHz / (2 * fast_half)
    input  wire [31:0] mbr_addr,         // block holding the MBR (0 normally)

    // ---------------- Status / results ----------------
    output wire        sd_init_done,
    output wire        sd_error,
    output wire [3:0]  sd_err_code,
    output wire        mounted,
    output wire        mount_busy,
    output wire        mount_error,
    output wire [3:0]  mount_err_code,
    output wire [31:0] part_lba,
    output wire [31:0] fat_lba,
    output wire [31:0] data_lba,
    output wire [31:0] root_cluster,
    output wire [7:0]  sectors_per_cluster,

    // ---------------- Debug ----------------
    output wire [15:0] dbg_cmd0,
    output wire [15:0] dbg_cmd8,
    output wire [15:0] dbg_acmd41,
    output wire [15:0] dbg_ocr,
    output wire [15:0] dbg_read,
    output wire [15:0] dbg_polls,

    // ---------------- SD card pins (Pmod) ----------------
    output wire        sd_sclk,
    output wire        sd_mosi,
    input  wire        sd_miso,
    output wire        sd_cs_n
);

    // ---------------- Controller <-> mount wires ----------------
    wire        sd_busy;
    wire        sd_start_read;
    wire [31:0] sd_block_addr;
    wire [7:0]  sd_data;
    wire        sd_data_valid;
    wire [8:0]  sd_byte_index;
    wire        sd_block_done;

    SD_controller #(
        .POWER_WAIT_CYCLES(POWER_WAIT_CYCLES),
        .ACMD41_TIMEOUT(ACMD41_TIMEOUT),
        .TOKEN_TIMEOUT(TOKEN_TIMEOUT),
        .SLOW_HALF(SLOW_HALF)
    ) sd (
        .clk(clk), .rst(rst),
        .reinit(reinit),
        .start_read(sd_start_read),
        .block_addr(sd_block_addr),
        .fast_half(fast_half),
        .init_done(sd_init_done),
        .busy(sd_busy),
        .error(sd_error),
        .err_code(sd_err_code),
        .data_out(sd_data),
        .data_valid(sd_data_valid),
        .byte_index(sd_byte_index),
        .block_done(sd_block_done),
        .dbg_cmd0(dbg_cmd0), .dbg_cmd8(dbg_cmd8), .dbg_acmd41(dbg_acmd41),
        .dbg_ocr(dbg_ocr), .dbg_read(dbg_read), .dbg_polls(dbg_polls),
        .sclk(sd_sclk), .mosi(sd_mosi), .miso(sd_miso), .cs_n(sd_cs_n)
    );

    // ---------------- Auto-mount when the card becomes ready ----------------
    reg init_done_d;
    always @(posedge clk or posedge rst) begin
        if (rst) init_done_d <= 1'b0;
        else     init_done_d <= sd_init_done;
    end
    wire auto_mount = sd_init_done & ~init_done_d;   // rising edge

    FAT32_mount fat (
        .clk(clk), .rst(rst),
        .mount_start(mount_start | auto_mount),
        .mbr_addr(mbr_addr),
        .mounted(mounted),
        .busy(mount_busy),
        .error(mount_error),
        .err_code(mount_err_code),
        .part_lba(part_lba),
        .fat_lba(fat_lba),
        .data_lba(data_lba),
        .root_cluster(root_cluster),
        .sectors_per_cluster(sectors_per_cluster),
        .sd_init_done(sd_init_done),
        .sd_busy(sd_busy),
        .sd_error(sd_error),
        .sd_start_read(sd_start_read),
        .sd_block_addr(sd_block_addr),
        .sd_data(sd_data),
        .sd_data_valid(sd_data_valid),
        .sd_byte_index(sd_byte_index),
        .sd_block_done(sd_block_done)
    );

endmodule
