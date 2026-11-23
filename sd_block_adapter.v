`timescale 1ns / 1ps
`default_nettype none
// Adapter: fat32_top's block-read interface  <->  SD_controller
//
//   fat32_top                    SD_controller
//   rd_start (pulse) ----------> start_read   (pulse, but IGNORED unless the
//   rd_lba   ------------------> block_addr    controller is idle + init_done)
//   rd_valid <------------------ data_valid   (pulse per byte, passthrough)
//   rd_byte  <------------------ data_out
//   rd_done  <------------------ block_done   (pulse, passthrough)
//   rd_err   <------------------ error        (LEVEL -> 1-cycle pulse)
//
// Why it exists:
//   1. SD_controller drops a start_read that arrives while it is busy, so
//      the request is latched here and issued once the controller is
//      ready (init_done && !busy). A dropped request would hang fat32_*.
//   2. SD_controller's error is held high until reinit; fat32_* expect a
//      single-cycle rd_err. The rising edge is turned into a pulse. A
//      request that arrives while the controller is already in error is
//      answered with rd_err immediately instead of hanging.
//   3. The LBA is registered with the request so block_addr is stable
//      while the controller samples it.

module sd_block_adapter (
    input  wire        clk,
    input  wire        rst,

    // ---------------- fat32_top side ----------------
    input  wire        rd_start,
    input  wire [31:0] rd_lba,
    output wire        rd_valid,
    output wire [7:0]  rd_byte,
    output wire        rd_done,
    output reg         rd_err,

    // ---------------- SD_controller side ----------------
    output reg         start_read,
    output reg  [31:0] block_addr,
    input  wire        init_done,
    input  wire        busy,
    input  wire        error,
    input  wire [7:0]  data_out,
    input  wire        data_valid,
    input  wire        block_done
);

    assign rd_valid = data_valid;
    assign rd_byte  = data_out;
    assign rd_done  = block_done;

    reg pending;    // request latched, not yet handed to the controller
    reg error_d;

    always @(posedge clk) begin
        if (rst) begin
            pending    <= 1'b0;
            error_d    <= 1'b0;
            start_read <= 1'b0;
            block_addr <= 32'd0;
            rd_err     <= 1'b0;
        end else begin
            start_read <= 1'b0;
            rd_err     <= 1'b0;
            error_d    <= error;

            if (rd_start) begin
                pending    <= 1'b1;
                block_addr <= rd_lba;
            end else if (pending) begin
                if (error) begin
                    rd_err  <= 1'b1;            // controller can't serve it
                    pending <= 1'b0;
                end else if (init_done && !busy) begin
                    start_read <= 1'b1;         // controller is in S_IDLE
                    pending    <= 1'b0;
                end
            end

            // controller failed mid-read
            if (error && !error_d)
                rd_err <= 1'b1;
        end
    end

endmodule
`default_nettype wire
