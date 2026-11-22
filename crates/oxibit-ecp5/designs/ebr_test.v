// Block RAM test for the ULX3S (25 MHz, 115200 8N1).
// - rom18: 8 x 16 initialised ROM (x18 DP16KD), two characters per word,
//   low byte first: "oxibit EBR x18. ".
// - rom9: 16 x 8 initialised ROM (x9): "x9 init ok" CR LF.
// - line: 256 x 8 RAM (x9). Typed characters are stored; Enter plays the
//   line back from the RAM, then CR LF. '?' sends the banner (both ROMs),
//   which also goes out once after configuration.
// led shows the stored line length.
module top(
    input        clk_25mhz,
    input        ftdi_txd,   // FPGA RX
    output       ftdi_rxd,   // FPGA TX
    output [7:0] led
);
    localparam DIV  = 217;
    localparam HALF = 108;

    // RX
    reg rx_s0 = 1, rx_s1 = 1;
    always @(posedge clk_25mhz) begin
        rx_s0 <= ftdi_txd;
        rx_s1 <= rx_s0;
    end
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
                if (rx_s1) rx_busy <= 0;
                rx_bit <= 1;
            end else if (rx_bit == 9) begin
                rx_busy <= 0;
                if (rx_s1) begin
                    rx_data <= rx_sh;
                    rx_stb  <= 1;
                end
            end else begin
                rx_sh  <= {rx_s1, rx_sh[7:1]};
                rx_bit <= rx_bit + 1;
            end
        end
    end

    // TX
    reg        tx_go   = 0;
    reg [7:0]  tx_byte = 0;
    reg        tx_busy = 0;
    reg [7:0]  tx_cnt  = 0;
    reg [3:0]  tx_bit  = 0;
    reg [9:0]  tx_sh   = 10'h3ff;
    always @(posedge clk_25mhz) begin
        if (!tx_busy) begin
            if (tx_go) begin
                tx_busy <= 1;
                tx_sh   <= {1'b1, tx_byte, 1'b0};
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

    // Memories
    reg [15:0] rom18 [0:7];
    reg [7:0]  rom9  [0:15];
    reg [7:0]  line  [0:255];
    initial begin
        rom18[0] = 16'h786f;
        rom18[1] = 16'h6269;
        rom18[2] = 16'h7469;
        rom18[3] = 16'h4520;
        rom18[4] = 16'h5242;
        rom18[5] = 16'h7820;
        rom18[6] = 16'h3831;
        rom18[7] = 16'h202e;
        rom9[0] = 8'h78;
        rom9[1] = 8'h39;
        rom9[2] = 8'h20;
        rom9[3] = 8'h69;
        rom9[4] = 8'h6e;
        rom9[5] = 8'h69;
        rom9[6] = 8'h74;
        rom9[7] = 8'h20;
        rom9[8] = 8'h6f;
        rom9[9] = 8'h6b;
        rom9[10] = 8'h0d;
        rom9[11] = 8'h0a;
        rom9[12] = 8'h00;
        rom9[13] = 8'h00;
        rom9[14] = 8'h00;
        rom9[15] = 8'h00;
    end
    reg [7:0]  idx = 0;
    reg [7:0]  len = 0;
    reg        line_we = 0;
    reg [15:0] rom18_q;
    reg [7:0]  rom9_q;
    reg [7:0]  line_q;
    always @(posedge clk_25mhz) rom18_q <= rom18[idx[3:1]];
    always @(posedge clk_25mhz) rom9_q <= rom9[idx[3:0]];
    always @(posedge clk_25mhz) begin
        if (line_we) line[len] <= rx_data;
        line_q <= line[idx];
    end

    // Sender: 0 idle, 1 fetch, 2 latch, 3 send, 4 wait for the byte.
    reg [2:0] st  = 1;
    reg [1:0] seg = 0;   // 0 rom18, 1 rom9, 2 line, 3 CR LF
    wire [7:0] rom18_byte = idx[0] ? rom18_q[15:8] : rom18_q[7:0];
    reg  [7:0] cur;
    always @(*) begin
        case (seg)
            2'd0: cur = rom18_byte;
            2'd1: cur = rom9_q;
            2'd2: cur = line_q;
            default: cur = (idx == 0) ? 8'h0d : 8'h0a;
        endcase
    end
    always @(posedge clk_25mhz) begin
        tx_go   <= 0;
        line_we <= 0;
        case (st)
            3'd0: begin
                if (line_we) len <= len + 1;
                if (rx_stb) begin
                    if (rx_data == 8'h3f) begin
                        seg <= 0; idx <= 0; st <= 1;
                    end else if (rx_data == 8'h0d) begin
                        seg <= 2; idx <= 0; st <= 1;
                    end else if (len != 8'hff) begin
                        line_we <= 1;
                    end
                end
            end
            3'd1: st <= 2;
            3'd2: begin
                if (seg == 0 && (idx == 16 || cur == 0)) begin
                    seg <= 1; idx <= 0; st <= 1;
                end else if (seg == 1 && (idx == 16 || cur == 0)) begin
                    st <= 0;
                end else if (seg == 2 && idx == len) begin
                    seg <= 3; idx <= 0; st <= 1;
                end else if (seg == 3 && idx == 2) begin
                    len <= 0; st <= 0;
                end else begin
                    tx_byte <= cur; tx_go <= 1; st <= 3;
                end
            end
            3'd3: st <= 4;
            default: begin
                if (!tx_busy && !tx_go) begin
                    idx <= idx + 1; st <= 1;
                end
            end
        endcase
    end
    assign led = len;
endmodule
