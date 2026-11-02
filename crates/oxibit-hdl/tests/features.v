module top (
  input clk, input rst, input rst_n,
  input [7:0] a, input [2:0] i, input [4:0] j, input [3:0] sel,
  input [15:0] wide, input [31:0] w32, input [1:0] k,
  output reg [3:0] zq, output reg [3:0] xq,
  output reg [7:0] r1, output reg [7:0] r2, output reg [7:0] hold,
  output o1, output [3:0] o2, output [3:0] o3, output o4,
  output reg [7:0] vl, output reg [15:0] vl2
);
  reg [7:0] up;
  reg [10:3] off;
  always @(*) begin
    casez (sel)
      4'b1???: zq = 4'd8;
      4'b01??: zq = 4'd4;
      4'b001z: zq = 4'd2;
      4'b0001: zq = 4'd1;
      default: zq = 4'd0;
    endcase
  end
  always @(*) begin
    casex (a[3:0])
      4'b1x0x: xq = a[7:4];
      4'b0?1?: xq = ~a[7:4];
      default: xq = 4'hf;
    endcase
  end
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      r1 <= 8'h5a;
    end else begin
      r1 <= r1 + a;
      hold <= a ^ r1;
    end
  end
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) r2 <= 8'd0;
    else if (sel[0]) r2 <= {r2[6:0], a[0]};
  end
  always @(posedge clk) off <= a;
  assign o1 = a[i];
  assign o2 = wide[j +: 4];
  assign o3 = wide[j -: 4];
  assign o4 = off[j];
  always @(posedge clk) begin
    vl[i] <= a[0];
    vl2[{k, 2'b01}] <= a[1];
  end
endmodule
