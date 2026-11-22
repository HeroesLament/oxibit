module top(input clk, input rst, input en, input [7:0] a, input [7:0] b, input [2:0] sel, output reg [7:0] y, output [8:0] sum, output lt, output reg [3:0] cnt);
  assign sum = a + b;
  assign lt = a < b;
  always @(posedge clk) begin
    if (rst) cnt <= 4'd0;
    else if (en) cnt <= cnt + 4'd1;
    case (sel)
      3'd0: y <= a & b;
      3'd1: y <= a | b;
      3'd2: y <= a - b;
      3'd3: y <= {a[3:0], b[7:4]};
      3'd4: y <= a ^ ~b;
      default: y <= y;
    endcase
  end
endmodule
