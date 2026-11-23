`timescale 1ns/1ps
// =============================================================================
// fat32_locate.v
// Mounts a FAT32 volume and finds one file (8.3 name) in the root directory.
//
// Flow:  MBR (LBA 0) -> boot sector / BPB -> compute geometry
//        -> scan root directory (follows the root dir's cluster chain)
//        -> outputs first_cluster + file_size + geometry for fat32_reader
//
// Parses bytes ON THE FLY as they stream out of the SD driver, so it needs
// no 512-byte sector buffer.
//
// SD driver interface assumed (adapt to yours):
//   rd_start  : 1-cycle pulse, start reading block rd_lba
//   rd_lba    : 32-bit BLOCK address (SDHC/SDXC style, not byte address)
//   rd_valid  : 1-cycle pulse per data byte (512 per block, in order)
//   rd_byte   : data byte, valid when rd_valid=1
//   rd_done   : 1-cycle pulse after the 512th byte has been delivered
//   rd_err    : 1-cycle pulse if the read failed (bad token / timeout)
// =============================================================================
module fat32_locate #(
    // 8.3 name, space padded, no dot, uppercase:  "VIDEO.BIN" -> "VIDEO   BIN"
    parameter [87:0] TARGET_NAME = "VIDEO   BIN"
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,          // pulse to begin mount + search

    // ---- to/from SD block driver ----
    output reg         rd_start,
    output reg  [31:0] rd_lba,
    input  wire        rd_valid,
    input  wire [7:0]  rd_byte,
    input  wire        rd_done,
    input  wire        rd_err,

    // ---- results ----
    output reg         busy,
    output reg         found,          // high (held) once file located
    output reg         error,          // high (held) on any failure
    output reg  [3:0]  err_code,       // see ERR_* below

    output reg  [31:0] first_cluster,
    output reg  [31:0] file_size,

    // geometry, needed by fat32_reader
    output reg  [31:0] fat_start,      // LBA of first FAT
    output reg  [31:0] data_start,     // LBA of cluster 2
    output reg  [2:0]  spc_shift,      // log2(sectors per cluster)

    // debug (handy on 7-seg / ILA)
    output reg  [31:0] part_start,
    output reg  [31:0] root_cluster
);
    // ---------------- error codes ----------------
    localparam ERR_NONE      = 4'd0;
    localparam ERR_SD        = 4'd1;  // SD driver reported read error
    localparam ERR_MBR_SIG   = 4'd2;  // sector 0 missing 0x55AA
    localparam ERR_BPB_SIG   = 4'd3;  // boot sector missing 0x55AA
    localparam ERR_BPS       = 4'd4;  // bytes/sector != 512
    localparam ERR_SPC       = 4'd5;  // sectors/cluster not a power of 2
    localparam ERR_NOT_FOUND = 4'd6;  // file not in root directory
    localparam ERR_BAD_CLUS  = 4'd7;  // bad cluster number in chain

    // ---------------- states ----------------
    localparam S_IDLE     = 4'd0;
    localparam S_MBR_REQ  = 4'd1;
    localparam S_MBR_WAIT = 4'd2;
    localparam S_BPB_REQ  = 4'd3;
    localparam S_BPB_WAIT = 4'd4;
    localparam S_CALC_FAT = 4'd5;   // data_start = fat_start + nfats*fatsz
    localparam S_CALC_SPC = 4'd6;   // spc -> shift amount
    localparam S_DIR_REQ  = 4'd7;
    localparam S_DIR_WAIT = 4'd8;
    localparam S_DIR_NEXT = 4'd9;
    localparam S_FAT_REQ  = 4'd10;
    localparam S_FAT_WAIT = 4'd11;
    localparam S_DONE     = 4'd12;
    localparam S_ERR      = 4'd13;

    reg [3:0]  state;
    reg [8:0]  idx;            // byte index within current sector (0..511)

    // values captured from MBR / BPB
    reg [7:0]  mbr_byte0;
    reg [7:0]  sig0;           // byte 510
    reg [15:0] bytes_per_sec;
    reg [7:0]  spc;            // sectors per cluster
    reg [15:0] rsvd_secs;
    reg [7:0]  num_fats;
    reg [31:0] fat_size;

    // directory walk
    reg [31:0] dir_cluster;
    reg [7:0]  sec_in_clus;
    reg [7:0]  fat_count;      // loop counter for nfats * fatsz

    // per-directory-entry scratch
    reg        name_match;
    reg [7:0]  ent_first;      // first byte of entry
    reg [15:0] ent_clus_hi;
    reg [15:0] ent_clus_lo;
    reg [31:0] ent_size;
    reg        ent_attr_ok;
    reg        dir_end;        // saw a 0x00 entry: no more entries
    reg        hit;            // found the file in this sector

    // FAT lookup scratch
    reg [31:0] next_cluster;

    wire [4:0] eoff = idx[4:0];          // offset inside 32-byte dir entry

    // byte i of TARGET_NAME (i = 0..10)
    function [7:0] tname;
        input [3:0] i;
        begin
            tname = TARGET_NAME[87 - 8*i -: 8];
        end
    endfunction

    // cluster -> first LBA of that cluster
    function [31:0] clus2lba;
        input [31:0] c;
        begin
            clus2lba = data_start + ((c - 32'd2) << spc_shift);
        end
    endfunction

    always @(posedge clk) begin
        rd_start <= 1'b0;

        if (rst) begin
            state    <= S_IDLE;
            busy     <= 1'b0;
            found    <= 1'b0;
            error    <= 1'b0;
            err_code <= ERR_NONE;
            idx      <= 9'd0;
            rd_lba   <= 32'd0;
        end else begin
            // any SD error while waiting on a read kills the mount
            if (rd_err && (state == S_MBR_WAIT || state == S_BPB_WAIT ||
                           state == S_DIR_WAIT || state == S_FAT_WAIT)) begin
                err_code <= ERR_SD;
                state    <= S_ERR;
            end else
            case (state)
            // -----------------------------------------------------------------
            S_IDLE: begin
                if (start) begin
                    busy     <= 1'b1;
                    found    <= 1'b0;
                    error    <= 1'b0;
                    err_code <= ERR_NONE;
                    state    <= S_MBR_REQ;
                end
            end

            // ---------------------------- MBR --------------------------------
            S_MBR_REQ: begin
                rd_lba   <= 32'd0;
                rd_start <= 1'b1;
                idx      <= 9'd0;
                state    <= S_MBR_WAIT;
            end

            S_MBR_WAIT: begin
                if (rd_valid) begin
                    idx <= idx + 9'd1;
                    case (idx)
                        9'h000: mbr_byte0        <= rd_byte;
                        // partition 1 entry starts at 0x1BE; start LBA at +8
                        9'h1C6: part_start[7:0]  <= rd_byte;
                        9'h1C7: part_start[15:8] <= rd_byte;
                        9'h1C8: part_start[23:16]<= rd_byte;
                        9'h1C9: part_start[31:24]<= rd_byte;
                        9'h1FE: sig0             <= rd_byte;
                        9'h1FF: if (sig0 != 8'h55 || rd_byte != 8'hAA)
                                    err_code <= ERR_MBR_SIG;
                        default: ;
                    endcase
                end
                if (rd_done) begin
                    if (err_code == ERR_MBR_SIG)
                        state <= S_ERR;
                    else begin
                        // "superfloppy" card: no MBR, sector 0 IS the boot
                        // sector (starts with a jump instruction EB/E9)
                        if (mbr_byte0 == 8'hEB || mbr_byte0 == 8'hE9)
                            part_start <= 32'd0;
                        state <= S_BPB_REQ;
                    end
                end
            end

            // ---------------------------- BPB --------------------------------
            S_BPB_REQ: begin
                rd_lba   <= part_start;
                rd_start <= 1'b1;
                idx      <= 9'd0;
                state    <= S_BPB_WAIT;
            end

            S_BPB_WAIT: begin
                if (rd_valid) begin
                    idx <= idx + 9'd1;
                    case (idx)
                        9'h00B: bytes_per_sec[7:0]  <= rd_byte;
                        9'h00C: bytes_per_sec[15:8] <= rd_byte;
                        9'h00D: spc                 <= rd_byte;
                        9'h00E: rsvd_secs[7:0]      <= rd_byte;
                        9'h00F: rsvd_secs[15:8]     <= rd_byte;
                        9'h010: num_fats            <= rd_byte;
                        9'h024: fat_size[7:0]       <= rd_byte;
                        9'h025: fat_size[15:8]      <= rd_byte;
                        9'h026: fat_size[23:16]     <= rd_byte;
                        9'h027: fat_size[31:24]     <= rd_byte;
                        9'h02C: root_cluster[7:0]   <= rd_byte;
                        9'h02D: root_cluster[15:8]  <= rd_byte;
                        9'h02E: root_cluster[23:16] <= rd_byte;
                        9'h02F: root_cluster[31:24] <= rd_byte;
                        9'h1FE: sig0                <= rd_byte;
                        9'h1FF: if (sig0 != 8'h55 || rd_byte != 8'hAA)
                                    err_code <= ERR_BPB_SIG;
                        default: ;
                    endcase
                end
                if (rd_done) begin
                    if (err_code == ERR_BPB_SIG)
                        state <= S_ERR;
                    else if (bytes_per_sec != 16'd512) begin
                        err_code <= ERR_BPS;
                        state    <= S_ERR;
                    end else begin
                        fat_start  <= part_start + {16'd0, rsvd_secs};
                        data_start <= part_start + {16'd0, rsvd_secs};
                        fat_count  <= 8'd0;
                        state      <= S_CALC_FAT;
                    end
                end
            end

            // ------------------------- geometry ------------------------------
            // add fat_size once per FAT (num_fats is 2 on basically every card,
            // so this takes 2 cycles and avoids a 32-bit multiplier)
            S_CALC_FAT: begin
                if (fat_count < num_fats) begin
                    data_start <= data_start + fat_size;
                    fat_count  <= fat_count + 8'd1;
                end else
                    state <= S_CALC_SPC;
            end

            S_CALC_SPC: begin
                case (spc)
                    8'd1:   spc_shift <= 3'd0;
                    8'd2:   spc_shift <= 3'd1;
                    8'd4:   spc_shift <= 3'd2;
                    8'd8:   spc_shift <= 3'd3;
                    8'd16:  spc_shift <= 3'd4;
                    8'd32:  spc_shift <= 3'd5;
                    8'd64:  spc_shift <= 3'd6;
                    8'd128: spc_shift <= 3'd7;
                    default: err_code <= ERR_SPC;
                endcase
                if (spc == 8'd1  || spc == 8'd2  || spc == 8'd4  || spc == 8'd8 ||
                    spc == 8'd16 || spc == 8'd32 || spc == 8'd64 || spc == 8'd128) begin
                    dir_cluster <= root_cluster;
                    sec_in_clus <= 8'd0;
                    dir_end     <= 1'b0;
                    hit         <= 1'b0;
                    state       <= S_DIR_REQ;
                end else
                    state <= S_ERR;
            end

            // ---------------------- root directory scan ----------------------
            S_DIR_REQ: begin
                rd_lba   <= clus2lba(dir_cluster) + {24'd0, sec_in_clus};
                rd_start <= 1'b1;
                idx      <= 9'd0;
                state    <= S_DIR_WAIT;
            end

            S_DIR_WAIT: begin
                if (rd_valid) begin
                    idx <= idx + 9'd1;
                    // only look at entries until we've found it / hit the end
                    if (!hit && !dir_end) begin
                        case (eoff)
                            5'd0: begin
                                ent_first  <= rd_byte;
                                name_match <= (rd_byte == tname(4'd0));
                            end
                            5'd1, 5'd2, 5'd3, 5'd4, 5'd5,
                            5'd6, 5'd7, 5'd8, 5'd9, 5'd10:
                                if (rd_byte != tname(eoff[3:0]))
                                    name_match <= 1'b0;
                            // attribute: reject LFN (0x0F), dirs (0x10),
                            // volume label (0x08)
                            5'd11: ent_attr_ok <= (rd_byte != 8'h0F) &&
                                                  ((rd_byte & 8'h18) == 8'h00);
                            5'd20: ent_clus_hi[7:0]  <= rd_byte;
                            5'd21: ent_clus_hi[15:8] <= rd_byte;
                            5'd26: ent_clus_lo[7:0]  <= rd_byte;
                            5'd27: ent_clus_lo[15:8] <= rd_byte;
                            5'd28: ent_size[7:0]     <= rd_byte;
                            5'd29: ent_size[15:8]    <= rd_byte;
                            5'd30: ent_size[23:16]   <= rd_byte;
                            5'd31: begin
                                // whole entry seen -> decide
                                if (ent_first == 8'h00)
                                    dir_end <= 1'b1;          // end of dir
                                else if (ent_first != 8'hE5 && name_match && ent_attr_ok) begin
                                    hit           <= 1'b1;
                                    first_cluster <= {ent_clus_hi, ent_clus_lo};
                                    file_size     <= {rd_byte, ent_size[23:0]};
                                end
                            end
                            default: ;
                        endcase
                    end
                end
                if (rd_done)
                    state <= S_DIR_NEXT;
            end

            S_DIR_NEXT: begin
                if (hit) begin
                    if (first_cluster < 32'd2) begin
                        err_code <= ERR_BAD_CLUS;   // empty file has cluster 0
                        state    <= S_ERR;
                    end else
                        state <= S_DONE;
                end else if (dir_end) begin
                    err_code <= ERR_NOT_FOUND;
                    state    <= S_ERR;
                end else if (sec_in_clus + 8'd1 == spc) begin
                    // end of this directory cluster -> follow FAT chain
                    sec_in_clus <= 8'd0;
                    state       <= S_FAT_REQ;
                end else begin
                    sec_in_clus <= sec_in_clus + 8'd1;
                    state       <= S_DIR_REQ;
                end
            end

            // --------------- FAT lookup for next directory cluster ----------
            // 128 FAT entries (4 bytes each) per 512-byte sector
            S_FAT_REQ: begin
                rd_lba   <= fat_start + (dir_cluster >> 7);
                rd_start <= 1'b1;
                idx      <= 9'd0;
                state    <= S_FAT_WAIT;
            end

            S_FAT_WAIT: begin
                if (rd_valid) begin
                    idx <= idx + 9'd1;
                    if (idx[8:2] == dir_cluster[6:0]) begin
                        case (idx[1:0])
                            2'd0: next_cluster[7:0]   <= rd_byte;
                            2'd1: next_cluster[15:8]  <= rd_byte;
                            2'd2: next_cluster[23:16] <= rd_byte;
                            2'd3: next_cluster[31:24] <= rd_byte & 8'h0F; // 28-bit
                        endcase
                    end
                end
                if (rd_done) begin
                    if (next_cluster >= 32'h0FFFFFF8) begin
                        err_code <= ERR_NOT_FOUND;   // end of dir chain
                        state    <= S_ERR;
                    end else if (next_cluster < 32'd2 || next_cluster >= 32'h0FFFFFF0) begin
                        err_code <= ERR_BAD_CLUS;
                        state    <= S_ERR;
                    end else begin
                        dir_cluster <= next_cluster;
                        state       <= S_DIR_REQ;
                    end
                end
            end

            // -----------------------------------------------------------------
            S_DONE: begin
                busy  <= 1'b0;
                found <= 1'b1;
                if (start) begin          // allow re-mount
                    found <= 1'b0;
                    busy  <= 1'b1;
                    state <= S_MBR_REQ;
                end
            end

            S_ERR: begin
                busy  <= 1'b0;
                error <= 1'b1;
                if (start) begin
                    error    <= 1'b0;
                    err_code <= ERR_NONE;
                    busy     <= 1'b1;
                    state    <= S_MBR_REQ;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
