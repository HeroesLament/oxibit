// UART echo plus free-running counter for the ULX3S (25 MHz, 115200 8N1).
// led shows the last received byte; holding btn[0] shows the counter instead.
module top(
    input        clk_25mhz,
    input        ftdi_txd,   // FPGA RX
    output       ftdi_rxd,   // FPGA TX
    input  [0:0] btn,
    output [7:0] led
);
    localparam DIV  = 217;   // 25e6 / 115200
    localparam HALF = 108;

    // free-running counter
    reg [27:0] ctr = 0;
    always @(posedge clk_25mhz) ctr <= ctr + 1;

    // RX synchroniser
    reg rx_s0 = 1, rx_s1 = 1;
    always @(posedge clk_25mhz) begin
        rx_s0 <= ftdi_txd;
        rx_s1 <= rx_s0;
    end

    // RX: wait for start edge, sample mid-bit
    reg        rx_busy = 0;
    reg [7:0]  rx_cnt  = 0;
    reg [3:0]  rx_bit  = 0;
    reg [7:0]  rx_sh   = 0;
    reg [7:0]  rx_data = 0;
    reg        rx_stb  = 0;
    always @(posedge clk_25mhz) begin
        rx_stb <= 0;
        if (!rx_busy) begin
            if (!rx_s1) begin
                rx_busy <= 1;
                rx_cnt  <= HALF;
                rx_bit  <= 0;
            end
        end else if (rx_cnt != 0) begin
            rx_cnt <= rx_cnt - 1;
        end else begin
            rx_cnt <= DIV - 1;
            if (rx_bit == 0) begin
                if (rx_s1) rx_busy <= 0;        // false start
                rx_bit <= 1;
            end else if (rx_bit == 9) begin
                rx_busy <= 0;
                if (rx_s1) begin               // valid stop bit
                    rx_data <= rx_sh;
                    rx_stb  <= 1;
                end
            end else begin
                rx_sh  <= {rx_s1, rx_sh[7:1]};
                rx_bit <= rx_bit + 1;
            end
        end
    end

    // TX: start, 8 data LSB first, stop
    reg        tx_busy = 0;
    reg [7:0]  tx_cnt  = 0;
    reg [3:0]  tx_bit  = 0;
    reg [9:0]  tx_sh   = 10'h3ff;
    always @(posedge clk_25mhz) begin
        if (!tx_busy) begin
            if (rx_stb) begin
                tx_busy <= 1;
                tx_sh   <= {1'b1, rx_data, 1'b0};
                tx_cnt  <= DIV - 1;
                tx_bit  <= 0;
            end
        end else if (tx_cnt != 0) begin
            tx_cnt <= tx_cnt - 1;
        end else begin
            tx_cnt <= DIV - 1;
            tx_sh  <= {1'b1, tx_sh[9:1]};
            if (tx_bit == 9) tx_busy <= 0;
            tx_bit <= tx_bit + 1;
        end
    end
    assign ftdi_rxd = tx_sh[0];

    assign led = btn[0] ? ctr[27:20] : rx_data;
endmodule
