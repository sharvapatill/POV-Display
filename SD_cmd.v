`timescale 1ns / 1ps
// SD card command engine
//   1. Builds the 6-byte command: {01, index, argument, checksum, 1}
//   2. Sends it one byte at a time through SPI_driver
//   3. Sends 0xFF up to 8 times until a reply byte (top bit 0) comes back
//   4. Reads 0-4 extra response bytes (4 for CMD8 and CMD58)
// Chip select is NOT handled here - the main controller owns it.

module SD_cmd (
    input  wire        clk,
    input  wire        rst,

    // ---------------- From the main controller ----------------
    input  wire        cmd_start,     // pulse to send a command
    input  wire [5:0]  cmd_index,     // command number (0, 8, 17, 55, ...)
    input  wire [31:0] cmd_arg,       // argument
    input  wire [2:0]  extra_bytes,   // response bytes after R1 (0 or 4)

    // ---------------- To the main controller ----------------
    output reg         cmd_done,      // 1-cycle pulse when finished
    output wire        cmd_busy,
    output reg  [7:0]  r1,            // reply byte
    output reg  [31:0] resp_data,     // extra response bytes (CMD8 / CMD58)
    output reg         timeout,       // no reply after 8 tries

    // ---------------- To/from SPI_driver ----------------
    output reg         spi_start,
    output reg  [7:0]  spi_tx,
    input  wire [7:0]  spi_rx,
    input  wire        spi_done
);

    // ------------------------------------------------------------------
    // Checksum (CRC7). The card only checks it on CMD0 and CMD8, but
    // computing it for every command costs almost nothing and means
    // no special cases. CMD0 -> 0x95, CMD8 (0x1AA) -> 0x87.
    // ------------------------------------------------------------------
    function [6:0] crc7(input [39:0] data);
        integer i;
        reg [6:0] c;
        reg       fb;
        begin
            c = 7'd0;
            for (i = 39; i >= 0; i = i - 1) begin
                fb = c[6] ^ data[i];
                c  = {c[5:0], 1'b0};
                if (fb) c = c ^ 7'h09;   // polynomial x^7 + x^3 + 1
            end
            crc7 = c;
        end
    endfunction

    wire [39:0] cmd_body  = {2'b01, cmd_index, cmd_arg};
    wire [47:0] cmd_frame = {cmd_body, crc7(cmd_body), 1'b1};

    // ------------------------------------------------------------------
    // States
    // ------------------------------------------------------------------
    localparam [2:0] IDLE       = 3'd0,
                     SEND       = 3'd1,   // start sending a command byte
                     WAIT_SEND  = 3'd2,
                     POLL       = 3'd3,   // send 0xFF, look for reply
                     WAIT_POLL  = 3'd4,
                     EXTRA      = 3'd5,   // send 0xFF, collect extra byte
                     WAIT_EXTRA = 3'd6,
                     FINISH     = 3'd7;

    reg [2:0]  state;
    reg [47:0] frame;        // shifts left 8 bits after each byte
    reg [2:0]  byte_count;   // command bytes sent (0-5)
    reg [2:0]  poll_count;   // reply attempts (0-7)
    reg [2:0]  extra_count;  // extra bytes read
    reg [2:0]  extra_total;  // latched copy of extra_bytes

    assign cmd_busy = (state != IDLE);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= IDLE;
            frame       <= 48'd0;
            byte_count  <= 3'd0;
            poll_count  <= 3'd0;
            extra_count <= 3'd0;
            extra_total <= 3'd0;
            cmd_done    <= 1'b0;
            r1          <= 8'hFF;
            resp_data   <= 32'd0;
            timeout     <= 1'b0;
            spi_start   <= 1'b0;
            spi_tx      <= 8'hFF;
        end else begin
            cmd_done  <= 1'b0;
            spi_start <= 1'b0;

            case (state)
                // ------------------------------------------------------
                IDLE: begin
                    if (cmd_start) begin
                        frame       <= cmd_frame;
                        extra_total <= extra_bytes;
                        byte_count  <= 3'd0;
                        poll_count  <= 3'd0;
                        extra_count <= 3'd0;
                        timeout     <= 1'b0;
                        r1          <= 8'hFF;
                        resp_data   <= 32'd0;
                        state       <= SEND;
                    end
                end

                // ---------------- Send the 6 command bytes ----------------
                SEND: begin
                    spi_tx    <= frame[47:40];
                    spi_start <= 1'b1;
                    state     <= WAIT_SEND;
                end

                WAIT_SEND: begin
                    if (spi_done) begin
                        frame <= {frame[39:0], 8'hFF};
                        if (byte_count == 3'd5) begin
                            state <= POLL;
                        end else begin
                            byte_count <= byte_count + 3'd1;
                            state      <= SEND;
                        end
                    end
                end

                // ---------------- Wait for the reply byte ----------------
                POLL: begin
                    spi_tx    <= 8'hFF;
                    spi_start <= 1'b1;
                    state     <= WAIT_POLL;
                end

                WAIT_POLL: begin
                    if (spi_done) begin
                        if (spi_rx[7] == 1'b0) begin          // real reply
                            r1 <= spi_rx;
                            if (extra_total == 3'd0) state <= FINISH;
                            else                     state <= EXTRA;
                        end else if (poll_count == 3'd7) begin // 8 tries, give up
                            timeout <= 1'b1;
                            state   <= FINISH;
                        end else begin
                            poll_count <= poll_count + 3'd1;
                            state      <= POLL;
                        end
                    end
                end

                // ---------------- Extra response bytes ----------------
                EXTRA: begin
                    spi_tx    <= 8'hFF;
                    spi_start <= 1'b1;
                    state     <= WAIT_EXTRA;
                end

                WAIT_EXTRA: begin
                    if (spi_done) begin
                        resp_data <= {resp_data[23:0], spi_rx};
                        if (extra_count == extra_total - 3'd1) begin
                            state <= FINISH;
                        end else begin
                            extra_count <= extra_count + 3'd1;
                            state       <= EXTRA;
                        end
                    end
                end

                // ------------------------------------------------------
                FINISH: begin
                    cmd_done <= 1'b1;
                    state    <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule