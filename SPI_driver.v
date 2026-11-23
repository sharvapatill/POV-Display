`timescale 1ns / 1ps
// SPI master, mode 0 (clock idles low, sample on rising edge, change on falling edge)
// One byte per transfer. Chip select is NOT handled here - the SD controller owns it.

module SPI_driver #(
    parameter SLOW_HALF = 250   // 100 MHz / (2 * 250) = 200 kHz  (startup)
)(
    input  wire       clk,       // 100 MHz system clock
    input  wire       rst,

    input  wire       fast,      // 0 = startup speed, 1 = full speed (only change while idle)
    input  wire [8:0] fast_half, // full speed = 100 MHz / (2 * fast_half): 2 = 25 MHz, 5 = 10 MHz
    input  wire       start,     // pulse to begin an 8-bit transfer
    input  wire [7:0] tx_data,   // byte to send, loaded on start
    output reg  [7:0] rx_data,   // byte received, valid when done pulses
    output reg        done,      // high for 1 cycle when transfer completes
    output wire       busy,      // high while a transfer is in progress

    input  wire       miso,
    output reg        mosi,
    output reg        sclk
);

    localparam [1:0] IDLE = 2'b00,
                     RISE = 2'b01,   // waiting to drive SCLK high (sample MISO)
                     FALL = 2'b10,   // waiting to drive SCLK low  (change MOSI)
                     DONE = 2'b11;

    reg [1:0] state;
    reg [7:0] tx_reg;
    reg [7:0] rx_reg;
    reg [2:0] bit_count;   // 0-7
    reg [8:0] div_count;

    wire [8:0] half = fast ? fast_half : SLOW_HALF;
    wire       tick = (div_count == half - 1);

    assign busy = (state != IDLE);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= IDLE;
            tx_reg    <= 8'hFF;
            rx_reg    <= 8'h00;
            bit_count <= 3'd0;
            div_count <= 9'd0;
            mosi      <= 1'b1;   // idle high
            sclk      <= 1'b0;   // mode 0: clock idles low
            rx_data   <= 8'h00;
            done      <= 1'b0;
        end else begin
            done <= 1'b0;

            // Clock divider: only counts during a transfer
            if (state == IDLE || tick)
                div_count <= 9'd0;
            else
                div_count <= div_count + 9'd1;

            case (state)
                IDLE: begin
                    sclk <= 1'b0;
                    if (start) begin
                        tx_reg    <= tx_data;
                        mosi      <= tx_data[7];   // first bit ready BEFORE first rising edge
                        bit_count <= 3'd0;
                        state     <= RISE;
                    end
                end

                RISE: if (tick) begin
                    sclk   <= 1'b1;
                    rx_reg <= {rx_reg[6:0], miso};  // sample MISO on rising edge
                    state  <= FALL;
                end

                FALL: if (tick) begin
                    sclk <= 1'b0;
                    if (bit_count == 3'd7) begin
                        state <= DONE;
                    end else begin
                        bit_count <= bit_count + 3'd1;
                        tx_reg    <= {tx_reg[6:0], 1'b1};
                        mosi      <= tx_reg[6];      // next bit, changed on falling edge
                        state     <= RISE;
                    end
                end

                DONE: begin
                    rx_data <= rx_reg;
                    done    <= 1'b1;
                    mosi    <= 1'b1;   // back to idle high
                    state   <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule