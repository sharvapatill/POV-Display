`timescale 1ns / 1ps
// FAT32 mount
//
//   Reads two blocks from the card through SD_controller and works out where
//   everything on the FAT32 partition lives:
//
//   1. Block 0 = MBR (partition table)
//        byte 450      : partition 1 type  (0x0B or 0x0C = FAT32)
//        bytes 454-457 : partition 1 start block (little-endian)
//        bytes 510-511 : 0x55 0xAA signature
//   2. First block of the partition = Volume Boot Record (VBR)
//        bytes 11-12   : bytes per sector (must be 512)
//        byte  13      : sectors per cluster
//        bytes 14-15   : reserved sectors (gap before the first FAT)
//        byte  16      : number of FATs (usually 2)
//        bytes 36-39   : sectors per FAT
//        bytes 44-47   : root directory cluster (usually 2)
//        bytes 510-511 : 0x55 0xAA signature
//   3. Computes
//        fat_lba  = part_lba + reserved
//        data_lba = fat_lba  + num_fats * fat_size
//      Any cluster N then lives at block: data_lba + (N - 2) * sectors_per_cluster
//
//   Bytes are grabbed as they stream past (data_valid + byte_index), so no
//   512-byte buffer is needed. FAT32 stores numbers little-endian: the
//   lowest-addressed byte is the least significant.
//
// Uses: SD_controller's read interface (start_read / block_addr / data_out /
//       data_valid / byte_index / block_done / busy / init_done / error)

module FAT32_mount (
    input  wire        clk,
    input  wire        rst,

    // ---------------- From the top module ----------------
    input  wire        mount_start,      // pulse to (re)mount
    input  wire [31:0] mbr_addr,         // block holding the MBR: 0 normally (other values = fault test)

    // ---------------- To the top module ----------------
    output reg         mounted,          // results below are valid
    output wire        busy,
    output reg         error,
    output reg  [3:0]  err_code,
    output reg  [31:0] part_lba,         // first block of the partition
    output reg  [31:0] fat_lba,          // first block of FAT #1
    output reg  [31:0] data_lba,         // first block of cluster 2
    output reg  [31:0] root_cluster,     // cluster where the root directory starts
    output reg  [7:0]  sectors_per_cluster,

    // ---------------- To/from SD_controller ----------------
    input  wire        sd_init_done,
    input  wire        sd_busy,
    input  wire        sd_error,
    output reg         sd_start_read,
    output reg  [31:0] sd_block_addr,
    input  wire [7:0]  sd_data,
    input  wire        sd_data_valid,
    input  wire [8:0]  sd_byte_index,
    input  wire        sd_block_done
);

    // ------------------------------------------------------------------
    // Error codes
    // ------------------------------------------------------------------
    localparam [3:0] ERR_NONE     = 4'd0,
                     ERR_SD       = 4'd1,  // SD controller reported an error
                     ERR_MBR_SIG  = 4'd2,  // block 0 has no 55 AA signature
                     ERR_NOT_FAT  = 4'd3,  // partition 1 is not FAT32 (type not 0B/0C)
                     ERR_VBR_SIG  = 4'd4,  // boot record has no 55 AA signature
                     ERR_FORMAT   = 4'd5;  // unsupported layout (not 512 B sectors, 0 FATs, ...)

    // ------------------------------------------------------------------
    // States
    // ------------------------------------------------------------------
    localparam [3:0] S_IDLE      = 4'd0,
                     S_WAIT_SD   = 4'd1,   // wait for card ready and controller idle
                     S_MBR_REQ   = 4'd2,
                     S_MBR_READ  = 4'd3,
                     S_MBR_CHECK = 4'd4,
                     S_VBR_WAIT  = 4'd5,   // wait for controller idle again
                     S_VBR_REQ   = 4'd6,
                     S_VBR_READ  = 4'd7,
                     S_VBR_CHECK = 4'd8,
                     S_CALC1     = 4'd9,
                     S_CALC2     = 4'd10,
                     S_DONE      = 4'd11,
                     S_ERROR     = 4'd12;

    reg [3:0] state;
    assign busy = (state != S_IDLE) && (state != S_DONE) && (state != S_ERROR);

    // fields captured while the blocks stream past
    reg [7:0]  part_type;
    reg [15:0] sig;              // bytes 510-511, {byte510, byte511}
    reg [15:0] bytes_per_sec;
    reg [15:0] reserved_secs;
    reg [7:0]  num_fats;
    reg [31:0] fat_size;
    reg [31:0] fat_total;        // num_fats * fat_size

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state               <= S_IDLE;
            mounted             <= 1'b0;
            error               <= 1'b0;
            err_code            <= ERR_NONE;
            part_lba            <= 32'd0;
            fat_lba             <= 32'd0;
            data_lba            <= 32'd0;
            root_cluster        <= 32'd0;
            sectors_per_cluster <= 8'd0;
            sd_start_read       <= 1'b0;
            sd_block_addr       <= 32'd0;
            part_type           <= 8'd0;
            sig                 <= 16'd0;
            bytes_per_sec       <= 16'd0;
            reserved_secs       <= 16'd0;
            num_fats            <= 8'd0;
            fat_size            <= 32'd0;
            fat_total           <= 32'd0;
        end else begin
            sd_start_read <= 1'b0;   // one-cycle pulse

            // A new mount request is accepted when not already mounting
            if (mount_start && !busy) begin
                mounted  <= 1'b0;
                error    <= 1'b0;
                err_code <= ERR_NONE;
                state    <= S_WAIT_SD;
            end else begin
                case (state)
                    S_IDLE, S_DONE, S_ERROR: ;   // wait for mount_start

                    // ---------------- Wait for the card ----------------
                    S_WAIT_SD: begin
                        if (sd_error) begin
                            err_code <= ERR_SD;
                            state    <= S_ERROR;
                        end else if (sd_init_done && !sd_busy)
                            state <= S_MBR_REQ;
                    end

                    // ---------------- Block 0: MBR ----------------
                    S_MBR_REQ: begin
                        sd_block_addr <= mbr_addr;
                        sd_start_read <= 1'b1;
                        sig           <= 16'd0;
                        part_type     <= 8'd0;
                        state         <= S_MBR_READ;
                    end

                    S_MBR_READ: begin
                        if (sd_data_valid) begin
                            case (sd_byte_index)
                                9'd450: part_type       <= sd_data;
                                9'd454: part_lba[7:0]   <= sd_data;
                                9'd455: part_lba[15:8]  <= sd_data;
                                9'd456: part_lba[23:16] <= sd_data;
                                9'd457: part_lba[31:24] <= sd_data;
                                9'd510: sig[15:8]       <= sd_data;
                                9'd511: sig[7:0]        <= sd_data;
                                default: ;
                            endcase
                        end
                        if (sd_error) begin
                            err_code <= ERR_SD;
                            state    <= S_ERROR;
                        end else if (sd_block_done)
                            state <= S_MBR_CHECK;
                    end

                    S_MBR_CHECK: begin
                        if (sig != 16'h55AA) begin
                            err_code <= ERR_MBR_SIG;
                            state    <= S_ERROR;
                        end else if (part_type != 8'h0B && part_type != 8'h0C) begin
                            err_code <= ERR_NOT_FAT;
                            state    <= S_ERROR;
                        end else
                            state <= S_VBR_WAIT;
                    end

                    // ---------------- Partition block: VBR ----------------
                    S_VBR_WAIT: begin
                        if (sd_error) begin
                            err_code <= ERR_SD;
                            state    <= S_ERROR;
                        end else if (!sd_busy)
                            state <= S_VBR_REQ;
                    end

                    S_VBR_REQ: begin
                        sd_block_addr <= part_lba;
                        sd_start_read <= 1'b1;
                        sig           <= 16'd0;
                        state         <= S_VBR_READ;
                    end

                    S_VBR_READ: begin
                        if (sd_data_valid) begin
                            case (sd_byte_index)
                                9'd11:  bytes_per_sec[7:0]   <= sd_data;
                                9'd12:  bytes_per_sec[15:8]  <= sd_data;
                                9'd13:  sectors_per_cluster  <= sd_data;
                                9'd14:  reserved_secs[7:0]   <= sd_data;
                                9'd15:  reserved_secs[15:8]  <= sd_data;
                                9'd16:  num_fats             <= sd_data;
                                9'd36:  fat_size[7:0]        <= sd_data;
                                9'd37:  fat_size[15:8]       <= sd_data;
                                9'd38:  fat_size[23:16]      <= sd_data;
                                9'd39:  fat_size[31:24]      <= sd_data;
                                9'd44:  root_cluster[7:0]    <= sd_data;
                                9'd45:  root_cluster[15:8]   <= sd_data;
                                9'd46:  root_cluster[23:16]  <= sd_data;
                                9'd47:  root_cluster[31:24]  <= sd_data;
                                9'd510: sig[15:8]            <= sd_data;
                                9'd511: sig[7:0]             <= sd_data;
                                default: ;
                            endcase
                        end
                        if (sd_error) begin
                            err_code <= ERR_SD;
                            state    <= S_ERROR;
                        end else if (sd_block_done)
                            state <= S_VBR_CHECK;
                    end

                    S_VBR_CHECK: begin
                        if (sig != 16'h55AA) begin
                            err_code <= ERR_VBR_SIG;
                            state    <= S_ERROR;
                        end else if (bytes_per_sec != 16'd512 || sectors_per_cluster == 8'd0 ||
                                     num_fats == 8'd0 || fat_size == 32'd0) begin
                            err_code <= ERR_FORMAT;
                            state    <= S_ERROR;
                        end else
                            state <= S_CALC1;
                    end

                    // ---------------- Work out the layout (2 cycles) ----------------
                    S_CALC1: begin
                        fat_lba   <= part_lba + {16'd0, reserved_secs};
                        fat_total <= fat_size * num_fats;
                        state     <= S_CALC2;
                    end

                    S_CALC2: begin
                        data_lba <= fat_lba + fat_total;
                        mounted  <= 1'b1;
                        state    <= S_DONE;
                    end

                    default: state <= S_ERROR;
                endcase

                // error flag follows the state
                if (state == S_ERROR) error <= 1'b1;
            end
        end
    end

endmodule