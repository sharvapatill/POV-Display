`timescale 1ns / 1ps
// SD card main controller (SPI mode, read-only)
//
//   - Runs the startup sequence after reset or a reinit pulse
//   - Owns chip select
//   - Switches the SPI driver between the command module and itself
//   - Reads one 512-byte block per start_read and streams it out
//
// Uses: SPI_driver.v, SD_cmd.v

module SD_controller #(
    parameter POWER_WAIT_CYCLES = 100_000,      // 1 ms   @ 100 MHz
    parameter ACMD41_TIMEOUT    = 100_000_000,  // 1 s    @ 100 MHz
    parameter TOKEN_TIMEOUT     = 10_000_000,   // 100 ms @ 100 MHz
    parameter SLOW_HALF         = 250,          // 200 kHz SCLK during startup
    parameter CMD0_RETRIES      = 20            // CMD0 attempts before giving up
)(
    input  wire        clk,         // 100 MHz
    input  wire        rst,

    // ---------------- From the top module ----------------
    input  wire        reinit,      // pulse to rerun startup (only accepted in IDLE or ERROR)
    input  wire        start_read,  // pulse to read one block (only accepted in IDLE)
    input  wire [31:0] block_addr,  // block number to read
    input  wire [8:0]  fast_half,   // SPI speed after startup = 100 MHz / (2 * fast_half)

    // ---------------- To the top module ----------------
    output reg         init_done,   // card is ready
    output wire        busy,        // doing something, don't send start_read
    output reg         error,
    output reg  [3:0]  err_code,
    output reg  [7:0]  data_out,    // streamed data byte
    output reg         data_valid,  // 1-cycle pulse: data_out has a new byte
    output reg  [8:0]  byte_index,  // position of data_out in the block (0-511)
    output reg         block_done,  // 1-cycle pulse: all 512 bytes delivered

    // ---------------- Debug ----------------
    // What the card answered to each command (shown on the 7-segment display).
    // FF = no answer (the card never pulled MISO low).
    output reg  [15:0] dbg_cmd0,    // {attempts, R1}           good: 01 01
    output reg  [15:0] dbg_cmd8,    // low 16 bits of the echo  good: 01AA
    output reg  [15:0] dbg_acmd41,  // {attempts, final R1}     good: xx 00
    output reg  [15:0] dbg_ocr,     // CMD58 OCR, upper 16 bits good: Cxxx (powered up + high capacity)
    output reg  [15:0] dbg_read,    // {CMD17 R1, token}        good: 00 FE
    output reg  [15:0] dbg_polls,   // bytes polled before the token (saturates)

    // ---------------- SD card pins ----------------
    output wire        sclk,
    output wire        mosi,
    input  wire        miso,
    output reg         cs_n
);

    // ------------------------------------------------------------------
    // Error codes
    // ------------------------------------------------------------------
    localparam [3:0] ERR_NONE   = 4'd0,
                     ERR_CMD0   = 4'd1,  // card not responding (check wiring)
                     ERR_CMD8   = 4'd2,  // old or unsupported card
                     ERR_ACMD41 = 4'd3,  // card never became ready
                     ERR_CMD58  = 4'd4,  // not a large (SDHC/SDXC) card
                     ERR_CMD17  = 4'd5,  // read command rejected
                     ERR_TOKEN  = 4'd6,  // 0xFE never arrived
                     ERR_DTOKEN = 4'd7;  // card sent an error marker instead of 0xFE

    // ------------------------------------------------------------------
    // States
    // ------------------------------------------------------------------
    localparam [4:0]
        // startup
        S_POWER_WAIT  = 5'd0,
        S_WAKE        = 5'd1,
        S_WAKE_WAIT   = 5'd2,
        S_CMD0        = 5'd3,
        S_CHK0        = 5'd4,
        S_CMD8        = 5'd5,
        S_CHK8        = 5'd6,
        S_CMD55       = 5'd7,
        S_CHK55       = 5'd8,
        S_ACMD41      = 5'd9,
        S_CHK41       = 5'd10,
        S_CMD58       = 5'd11,
        S_CHK58       = 5'd12,
        S_GO_FAST     = 5'd13,
        // running
        S_IDLE        = 5'd14,
        S_CHK17       = 5'd15,
        S_TOKEN       = 5'd16,
        S_TOKEN_WAIT  = 5'd17,
        S_DATA        = 5'd18,
        S_DATA_WAIT   = 5'd19,
        S_CRC         = 5'd20,
        S_CRC_WAIT    = 5'd21,
        S_BLOCK_DONE  = 5'd22,
        // shared "send one command" routine
        C_BEGIN       = 5'd23,   // CS low, send one 0xFF
        C_PRE         = 5'd24,   // wait for that byte, then hand SPI to command module
        C_RUN         = 5'd25,   // wait for cmd_done
        C_FINISH      = 5'd26,   // CS high, send one 0xFF
        C_FINISH_WAIT = 5'd27,
        // error
        S_ERROR       = 5'd28;

    reg [4:0]  state;
    reg [4:0]  check_state;   // where to go after the command module finishes
    reg [4:0]  after_state;   // where to go after C_FINISH
    reg [26:0] timer;         // shared timer (27 bits holds 100,000,000)
    reg [9:0]  byte_cnt;      // wake bytes, data bytes, CRC bytes
    reg [4:0]  cmd0_tries;    // CMD0 attempts so far (cards often ignore the first)
    reg [7:0]  acmd41_tries;  // ACMD41 attempts so far (saturates at 255)
    reg        fast;          // SPI speed select

    assign busy = (state != S_IDLE) && (state != S_ERROR);

    // ------------------------------------------------------------------
    // Command module
    // ------------------------------------------------------------------
    reg         cmd_start;
    reg  [5:0]  cmd_index;
    reg  [31:0] cmd_arg;
    reg  [2:0]  cmd_extra;
    wire        cmd_done, cmd_busy, cmd_timeout;
    wire [7:0]  r1;
    wire [31:0] resp_data;

    wire        cmd_spi_start;
    wire [7:0]  cmd_spi_tx;

    // ------------------------------------------------------------------
    // SPI switch: command module OR this controller drives the SPI driver.
    // The controller only ever sends 0xFF, so its side is fixed.
    // ------------------------------------------------------------------
    reg         use_cmd;     // 1 = command module, 0 = controller
    reg         ctl_start;   // controller's own "send 0xFF" pulse

    wire        spi_start = use_cmd ? cmd_spi_start : ctl_start;
    wire [7:0]  spi_tx    = use_cmd ? cmd_spi_tx    : 8'hFF;
    wire [7:0]  spi_rx;
    wire        spi_done;
    wire        spi_busy;

    SPI_driver #(
        .SLOW_HALF(SLOW_HALF)
    ) spi (
        .clk(clk), .rst(rst), .fast(fast), .fast_half(fast_half),
        .start(spi_start), .tx_data(spi_tx),
        .rx_data(spi_rx), .done(spi_done), .busy(spi_busy),
        .miso(miso), .mosi(mosi), .sclk(sclk)
    );

    SD_cmd cmd (
        .clk(clk), .rst(rst),
        .cmd_start(cmd_start), .cmd_index(cmd_index), .cmd_arg(cmd_arg),
        .extra_bytes(cmd_extra),
        .cmd_done(cmd_done), .cmd_busy(cmd_busy), .r1(r1),
        .resp_data(resp_data), .timeout(cmd_timeout),
        .spi_start(cmd_spi_start), .spi_tx(cmd_spi_tx),
        .spi_rx(spi_rx), .spi_done(spi_done)
    );

    // ------------------------------------------------------------------
    // Main state machine
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= S_POWER_WAIT;
            check_state <= S_IDLE;
            after_state <= S_IDLE;
            timer       <= 27'd0;
            byte_cnt    <= 10'd0;
            cmd0_tries  <= 5'd0;
            fast        <= 1'b0;
            cs_n        <= 1'b1;
            use_cmd     <= 1'b0;
            ctl_start   <= 1'b0;
            cmd_start   <= 1'b0;
            cmd_index   <= 6'd0;
            cmd_arg     <= 32'd0;
            cmd_extra   <= 3'd0;
            init_done   <= 1'b0;
            error       <= 1'b0;
            err_code    <= ERR_NONE;
            data_out    <= 8'd0;
            data_valid  <= 1'b0;
            byte_index  <= 9'd0;
            block_done  <= 1'b0;
            acmd41_tries <= 8'd0;
            dbg_cmd0    <= 16'h0000;
            dbg_cmd8    <= 16'h0000;
            dbg_acmd41  <= 16'h0000;
            dbg_ocr     <= 16'h0000;
            dbg_read    <= 16'h0000;
            dbg_polls   <= 16'h0000;
        end else begin
            // one-cycle pulses default low
            ctl_start  <= 1'b0;
            cmd_start  <= 1'b0;
            data_valid <= 1'b0;
            block_done <= 1'b0;

            // shared timer always counts; states reset it when they need it
            timer <= timer + 27'd1;

            case (state)
                // ======================================================
                // STARTUP
                // ======================================================
                S_POWER_WAIT: begin
                    cs_n <= 1'b1;
                    if (timer >= POWER_WAIT_CYCLES - 1) begin
                        byte_cnt     <= 10'd0;
                        cmd0_tries   <= 5'd0;
                        acmd41_tries <= 8'd0;
                        dbg_cmd0     <= 16'h0000;   // fresh readings for this startup
                        dbg_cmd8     <= 16'h0000;
                        dbg_acmd41   <= 16'h0000;
                        dbg_ocr      <= 16'h0000;
                        dbg_read     <= 16'h0000;
                        dbg_polls    <= 16'h0000;
                        state    <= S_WAKE;
                    end
                end

                // 10 x 0xFF with CS high = 80 clocks (card needs 74)
                S_WAKE: begin
                    ctl_start <= 1'b1;
                    state     <= S_WAKE_WAIT;
                end

                S_WAKE_WAIT: begin
                    if (spi_done) begin
                        if (byte_cnt == 10'd9) state <= S_CMD0;
                        else begin
                            byte_cnt <= byte_cnt + 10'd1;
                            state    <= S_WAKE;
                        end
                    end
                end

                // ---------------- CMD0: reset into SPI mode ----------------
                S_CMD0: begin
                    cmd_index   <= 6'd0;
                    cmd_arg     <= 32'd0;
                    cmd_extra   <= 3'd0;
                    check_state <= S_CHK0;
                    state       <= C_BEGIN;
                end

                S_CHK0: begin
                    dbg_cmd0 <= {3'd0, cmd0_tries + 5'd1, r1};
                    if (!cmd_timeout && r1 == 8'h01) begin
                        after_state <= S_CMD8;
                        state       <= C_FINISH;
                    end else if (cmd0_tries < CMD0_RETRIES - 1) begin
                        cmd0_tries  <= cmd0_tries + 5'd1;   // card not awake yet, try again
                        after_state <= S_CMD0;
                        state       <= C_FINISH;
                    end else begin
                        err_code <= ERR_CMD0;
                        state    <= S_ERROR;
                    end
                end

                // ---------------- CMD8: voltage / version check ----------------
                S_CMD8: begin
                    cmd_index   <= 6'd8;
                    cmd_arg     <= 32'h0000_01AA;
                    cmd_extra   <= 3'd4;
                    check_state <= S_CHK8;
                    state       <= C_BEGIN;
                end

                S_CHK8: begin
                    dbg_cmd8 <= resp_data[15:0];
                    if (!cmd_timeout && r1 == 8'h01 && resp_data[11:0] == 12'h1AA) begin
                        timer       <= 27'd0;      // start the ACMD41 1-second clock
                        after_state <= S_CMD55;
                        state       <= C_FINISH;
                    end else begin
                        err_code <= ERR_CMD8;
                        state    <= S_ERROR;
                    end
                end

                // ---------------- CMD55 + ACMD41 loop ----------------
                S_CMD55: begin
                    cmd_index   <= 6'd55;
                    cmd_arg     <= 32'd0;
                    cmd_extra   <= 3'd0;
                    check_state <= S_CHK55;
                    state       <= C_BEGIN;
                end

                S_CHK55: begin
                    // 0x01 while starting up (0x00 also fine)
                    if (!cmd_timeout && r1[7:1] == 7'd0) begin
                        after_state <= S_ACMD41;
                        state       <= C_FINISH;
                    end else begin
                        err_code <= ERR_ACMD41;
                        state    <= S_ERROR;
                    end
                end

                S_ACMD41: begin
                    cmd_index   <= 6'd41;
                    cmd_arg     <= 32'h4000_0000;  // "I support large cards"
                    cmd_extra   <= 3'd0;
                    check_state <= S_CHK41;
                    state       <= C_BEGIN;
                end

                S_CHK41: begin
                    if (acmd41_tries != 8'hFF) acmd41_tries <= acmd41_tries + 8'd1;
                    dbg_acmd41 <= {(acmd41_tries == 8'hFF) ? 8'hFF : acmd41_tries + 8'd1, r1};
                    if (!cmd_timeout && r1 == 8'h00) begin
                        after_state <= S_CMD58;              // card ready
                        state       <= C_FINISH;
                    end else if (!cmd_timeout && r1 == 8'h01 && timer < ACMD41_TIMEOUT) begin
                        after_state <= S_CMD55;              // still busy, try again
                        state       <= C_FINISH;
                    end else begin
                        err_code <= ERR_ACMD41;
                        state    <= S_ERROR;
                    end
                end

                // ---------------- CMD58: read card settings ----------------
                S_CMD58: begin
                    cmd_index   <= 6'd58;
                    cmd_arg     <= 32'd0;
                    cmd_extra   <= 3'd4;
                    check_state <= S_CHK58;
                    state       <= C_BEGIN;
                end

                S_CHK58: begin
                    dbg_ocr <= resp_data[31:16];
                    // bit 31 = powered up, bit 30 = large card (block addressing)
                    if (!cmd_timeout && r1 == 8'h00 && resp_data[31] && resp_data[30]) begin
                        after_state <= S_GO_FAST;
                        state       <= C_FINISH;
                    end else begin
                        err_code <= ERR_CMD58;
                        state    <= S_ERROR;
                    end
                end

                S_GO_FAST: begin
                    fast      <= 1'b1;
                    init_done <= 1'b1;
                    state     <= S_IDLE;
                end

                // ======================================================
                // RUNNING
                // ======================================================
                S_IDLE: begin
                    if (reinit) begin
                        state     <= S_POWER_WAIT;
                        timer     <= 27'd0;
                        fast      <= 1'b0;
                        init_done <= 1'b0;
                        cs_n      <= 1'b1;
                    end else if (start_read && init_done) begin
                        cmd_index   <= 6'd17;
                        cmd_arg     <= block_addr;
                        cmd_extra   <= 3'd0;
                        check_state <= S_CHK17;
                        state       <= C_BEGIN;
                    end
                end

                S_CHK17: begin
                    dbg_read  <= {r1, 8'h00};
                    dbg_polls <= 16'h0000;
                    if (!cmd_timeout && r1 == 8'h00) begin
                        timer <= 27'd0;          // start the 100 ms token clock
                        state <= S_TOKEN;        // CS stays LOW for the data
                    end else begin
                        err_code <= ERR_CMD17;
                        state    <= S_ERROR;
                    end
                end

                // ---------------- Wait for 0xFE ----------------
                S_TOKEN: begin
                    ctl_start <= 1'b1;
                    state     <= S_TOKEN_WAIT;
                end

                S_TOKEN_WAIT: begin
                    if (spi_done) begin
                        dbg_read[7:0] <= spi_rx;
                        if (dbg_polls != 16'hFFFF) dbg_polls <= dbg_polls + 16'd1;
                        if (spi_rx == 8'hFE) begin
                            byte_cnt <= 10'd0;
                            state    <= S_DATA;         // next byte is data byte 0
                        end else if (spi_rx == 8'hFF) begin
                            if (timer >= TOKEN_TIMEOUT) begin
                                err_code <= ERR_TOKEN;
                                state    <= S_ERROR;
                            end else
                                state <= S_TOKEN;       // card still looking
                        end else begin
                            err_code <= ERR_DTOKEN;     // card sent an error marker
                            state    <= S_ERROR;
                        end
                    end
                end

                // ---------------- 512 data bytes ----------------
                S_DATA: begin
                    ctl_start <= 1'b1;
                    state     <= S_DATA_WAIT;
                end

                S_DATA_WAIT: begin
                    if (spi_done) begin
                        data_out   <= spi_rx;
                        byte_index <= byte_cnt[8:0];
                        data_valid <= 1'b1;
                        if (byte_cnt == 10'd511) begin
                            byte_cnt <= 10'd0;
                            state    <= S_CRC;
                        end else begin
                            byte_cnt <= byte_cnt + 10'd1;
                            state    <= S_DATA;
                        end
                    end
                end

                // ---------------- 2 checksum bytes, thrown away ----------------
                S_CRC: begin
                    ctl_start <= 1'b1;
                    state     <= S_CRC_WAIT;
                end

                S_CRC_WAIT: begin
                    if (spi_done) begin
                        if (byte_cnt == 10'd1) begin
                            after_state <= S_BLOCK_DONE;
                            state       <= C_FINISH;    // CS high + one 0xFF
                        end else begin
                            byte_cnt <= byte_cnt + 10'd1;
                            state    <= S_CRC;
                        end
                    end
                end

                S_BLOCK_DONE: begin
                    block_done <= 1'b1;
                    state      <= S_IDLE;
                end

                // ======================================================
                // SHARED "SEND ONE COMMAND" ROUTINE
                // ======================================================
                C_BEGIN: begin
                    cs_n      <= 1'b0;          // select the card
                    ctl_start <= 1'b1;          // one 0xFF before the command
                    state     <= C_PRE;
                end

                C_PRE: begin
                    if (spi_done) begin
                        use_cmd   <= 1'b1;      // hand SPI to the command module
                        cmd_start <= 1'b1;
                        state     <= C_RUN;
                    end
                end

                C_RUN: begin
                    if (cmd_done) begin
                        use_cmd <= 1'b0;        // take SPI back
                        state   <= check_state;
                    end
                end

                C_FINISH: begin
                    cs_n      <= 1'b1;          // deselect
                    ctl_start <= 1'b1;          // one 0xFF so the card lets go of MISO
                    state     <= C_FINISH_WAIT;
                end

                C_FINISH_WAIT: begin
                    if (spi_done) state <= after_state;
                end

                // ======================================================
                // ERROR: wait for reinit
                // ======================================================
                S_ERROR: begin
                    error     <= 1'b1;
                    cs_n      <= 1'b1;
                    init_done <= 1'b0;
                    if (reinit) begin
                        error    <= 1'b0;
                        err_code <= ERR_NONE;
                        fast     <= 1'b0;
                        timer    <= 27'd0;
                        state    <= S_POWER_WAIT;
                    end
                end

                default: state <= S_ERROR;
            endcase
        end
    end

endmodule
