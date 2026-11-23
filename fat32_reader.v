`timescale 1ns/1ps
// =============================================================================
// fat32_reader.v
// Streams a file's contents out, starting from its first cluster and
// following the FAT cluster chain until file_size bytes have been delivered.
//
// Output is a byte stream (out_valid/out_byte) + a sector_done pulse, so your
// frame buffer can write straight into BRAM with no extra buffering.
//
// Flow control: the reader only requests the next sector while ok_to_read=1.
// Tie it high to stream flat out, or drive it from your double-buffer logic
// ("there's room for another 512 bytes") to pause between sectors.
//
// Same SD driver interface as fat32_locate.v.
// =============================================================================
module fat32_reader (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,          // pulse: begin reading the file

    // from fat32_locate
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size,
    input  wire [31:0] fat_start,
    input  wire [31:0] data_start,
    input  wire [2:0]  spc_shift,

    // flow control from frame buffer
    input  wire        ok_to_read,

    // ---- to/from SD block driver ----
    output reg         rd_start,
    output reg  [31:0] rd_lba,
    input  wire        rd_valid,
    input  wire [7:0]  rd_byte,
    input  wire        rd_done,
    input  wire        rd_err,

    // ---- file data out ----
    output reg         out_valid,      // 1-cycle pulse per file byte
    output reg  [7:0]  out_byte,
    output reg         sector_done,    // pulse after each sector's bytes
    output reg         file_done,      // held high when whole file delivered
    output reg         busy,
    output reg         error,
    output reg  [1:0]  err_code,       // 1=SD read error, 2=bad cluster/chain
    output reg  [31:0] bytes_sent      // running count (debug/progress)
);
    localparam S_IDLE     = 3'd0;
    localparam S_SEC_REQ  = 3'd1;
    localparam S_SEC_WAIT = 3'd2;
    localparam S_SEC_NEXT = 3'd3;
    localparam S_FAT_REQ  = 3'd4;
    localparam S_FAT_WAIT = 3'd5;
    localparam S_DONE     = 3'd6;
    localparam S_ERR      = 3'd7;

    reg [2:0]  state;
    reg [8:0]  idx;            // byte index inside current sector
    reg [31:0] cluster;        // current cluster
    reg [7:0]  sec_in_clus;    // sector index inside current cluster
    reg [31:0] remaining;      // file bytes still to deliver
    reg [31:0] next_cluster;

    wire [7:0] spc_minus1 = (8'd1 << spc_shift) - 8'd1;

    always @(posedge clk) begin
        rd_start    <= 1'b0;
        out_valid   <= 1'b0;
        sector_done <= 1'b0;

        if (rst) begin
            state     <= S_IDLE;
            busy      <= 1'b0;
            error     <= 1'b0;
            err_code  <= 2'd0;
            file_done <= 1'b0;
            idx       <= 9'd0;
            rd_lba    <= 32'd0;
            bytes_sent<= 32'd0;
        end else begin
            if (rd_err && (state == S_SEC_WAIT || state == S_FAT_WAIT)) begin
                err_code <= 2'd1;
                state    <= S_ERR;
            end else if (start && (state == S_IDLE || state == S_DONE || state == S_ERR)) begin
                // (re)start: also lets you replay the file by pulsing start
                busy        <= 1'b1;
                error       <= 1'b0;
                err_code    <= 2'd0;
                file_done   <= 1'b0;
                bytes_sent  <= 32'd0;
                cluster     <= first_cluster;
                sec_in_clus <= 8'd0;
                remaining   <= file_size;
                if (first_cluster < 32'd2) begin
                    err_code <= 2'd2;
                    state    <= S_ERR;
                end else if (file_size == 32'd0)
                    state <= S_DONE;
                else
                    state <= S_SEC_REQ;
            end else
            case (state)
            S_IDLE: ;   // waits for start (handled above)

            // ------------------- read one data sector ------------------------
            S_SEC_REQ: begin
                if (ok_to_read) begin
                    rd_lba   <= data_start + ((cluster - 32'd2) << spc_shift)
                                           + {24'd0, sec_in_clus};
                    rd_start <= 1'b1;
                    idx      <= 9'd0;
                    state    <= S_SEC_WAIT;
                end
            end

            S_SEC_WAIT: begin
                if (rd_valid) begin
                    idx <= idx + 9'd1;
                    // last sector is usually partial: drop the slack bytes
                    if (remaining != 32'd0) begin
                        out_valid  <= 1'b1;
                        out_byte   <= rd_byte;
                        remaining  <= remaining - 32'd1;
                        bytes_sent <= bytes_sent + 32'd1;
                    end
                end
                if (rd_done) begin
                    sector_done <= 1'b1;
                    state       <= S_SEC_NEXT;
                end
            end

            S_SEC_NEXT: begin
                if (remaining == 32'd0)
                    state <= S_DONE;
                else if (sec_in_clus == spc_minus1) begin
                    sec_in_clus <= 8'd0;
                    state       <= S_FAT_REQ;     // cluster used up
                end else begin
                    sec_in_clus <= sec_in_clus + 8'd1;
                    state       <= S_SEC_REQ;
                end
            end

            // ------------------- FAT lookup: next cluster --------------------
            S_FAT_REQ: begin
                rd_lba   <= fat_start + (cluster >> 7);
                rd_start <= 1'b1;
                idx      <= 9'd0;
                state    <= S_FAT_WAIT;
            end

            S_FAT_WAIT: begin
                if (rd_valid) begin
                    idx <= idx + 9'd1;
                    if (idx[8:2] == cluster[6:0]) begin
                        case (idx[1:0])
                            2'd0: next_cluster[7:0]   <= rd_byte;
                            2'd1: next_cluster[15:8]  <= rd_byte;
                            2'd2: next_cluster[23:16] <= rd_byte;
                            2'd3: next_cluster[31:24] <= rd_byte & 8'h0F;
                        endcase
                    end
                end
                if (rd_done) begin
                    // still have bytes to send, so the chain must continue;
                    // EOC / free / bad / reserved here means a corrupt chain
                    if (next_cluster < 32'd2 || next_cluster >= 32'h0FFFFFF0) begin
                        err_code <= 2'd2;
                        state    <= S_ERR;
                    end else begin
                        cluster <= next_cluster;
                        state   <= S_SEC_REQ;
                    end
                end
            end

            S_DONE: begin
                busy      <= 1'b0;
                file_done <= 1'b1;
            end

            S_ERR: begin
                busy  <= 1'b0;
                error <= 1'b1;
            end

            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
