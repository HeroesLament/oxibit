module top(input clk_25mhz, output [7:0] led);
  reg [27:0] ctr = 0;
  always @(posedge clk_25mhz) ctr <= ctr + 1;
  assign led = ctr[27:20];
endmodule
