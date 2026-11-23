`timescale 1ns/1ps
// =============================================================================
// fat32_top.v
// Glues fat32_locate and fat32_reader to ONE SD block driver.
//
//   SD driver <-> [mux] <-> fat32_locate --(first_cluster, geometry)--> fat32_reader --> frame buffer
//
// Sequencing: pulse `start` -> locate mounts the card and finds the file ->
// reader is kicked automatically and streams the file out.
// While locate is running it owns the SD request lines; after `found`
// the reader owns them. Read data (rd_valid/rd_byte/rd_done/rd_err) is
// broadcast to both; each module only listens in its own wait states.
// =============================================================================
module fat32_top #(
    parameter [87:0] TARGET_NAME = "VIDEO   BIN"
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,          // pulse to mount + stream the file
    input  wire        replay,         // pulse to stream the file again
    input  wire        ok_to_read,     // flow control from frame buffer

    // ---- SD block driver ----
    output wire        sd_rd_start,
    output wire [31:0] sd_rd_lba,
    input  wire        sd_rd_valid,
    input  wire [7:0]  sd_rd_byte,
    input  wire        sd_rd_done,
    input  wire        sd_rd_err,

    // ---- file data out ----
    output wire        out_valid,
    output wire [7:0]  out_byte,
    output wire        sector_done,
    output wire        file_done,

    // ---- status ----
    output wire        found,
    output wire        error,          // either module failed
    output wire [3:0]  locate_err,
    output wire [1:0]  reader_err,
    output wire [31:0] first_cluster,
    output wire [31:0] file_size,
    output wire [31:0] bytes_sent,
    output wire [31:0] part_start      // debug: should match your hex editor
);
    // locate outputs
    wire        loc_rd_start;
    wire [31:0] loc_rd_lba;
    wire        loc_busy, loc_error;
    wire [31:0] fat_start, data_start, root_cluster;
    wire [2:0]  spc_shift;

    // reader outputs
    wire        rdr_rd_start;
    wire [31:0] rdr_rd_lba;
    wire        rdr_busy, rdr_error;

    fat32_locate #(.TARGET_NAME(TARGET_NAME)) u_locate (
        .clk(clk), .rst(rst), .start(start),
        .rd_start(loc_rd_start), .rd_lba(loc_rd_lba),
        .rd_valid(sd_rd_valid), .rd_byte(sd_rd_byte),
        .rd_done(sd_rd_done),   .rd_err(sd_rd_err),
        .busy(loc_busy), .found(found), .error(loc_error), .err_code(locate_err),
        .first_cluster(first_cluster), .file_size(file_size),
        .fat_start(fat_start), .data_start(data_start), .spc_shift(spc_shift),
        .part_start(part_start), .root_cluster(root_cluster)
    );

    // kick the reader on the rising edge of `found`, or on replay
    reg found_d;
    always @(posedge clk) found_d <= rst ? 1'b0 : found;
    wire reader_go = (found & ~found_d) | (replay & found);

    fat32_reader u_reader (
        .clk(clk), .rst(rst), .start(reader_go),
        .first_cluster(first_cluster), .file_size(file_size),
        .fat_start(fat_start), .data_start(data_start), .spc_shift(spc_shift),
        .ok_to_read(ok_to_read),
        .rd_start(rdr_rd_start), .rd_lba(rdr_rd_lba),
        .rd_valid(sd_rd_valid), .rd_byte(sd_rd_byte),
        .rd_done(sd_rd_done),   .rd_err(sd_rd_err),
        .out_valid(out_valid), .out_byte(out_byte),
        .sector_done(sector_done), .file_done(file_done),
        .busy(rdr_busy), .error(rdr_error), .err_code(reader_err),
        .bytes_sent(bytes_sent)
    );

    // ---- SD request mux: locate owns the bus until the file is found ----
    assign sd_rd_start = found ? rdr_rd_start : loc_rd_start;
    assign sd_rd_lba   = found ? rdr_rd_lba   : loc_rd_lba;

    assign error = loc_error | rdr_error;
endmodule
