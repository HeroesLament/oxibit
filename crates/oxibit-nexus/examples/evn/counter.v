// LED counter for the CrossLink-NX EVN board.
//
// The internal oscillator runs at 450 MHz / (HF_CLK_DIV + 1) = 1.76 MHz;
// a 34-bit counter shows bits 33..20 on the 14 LEDs (active low), so LED0
// toggles at about 0.84 Hz. Holding the GSRN button (low) clears it.
module top(input gsrn, output [13:0] led);
	wire clk;

	OSC_CORE #(.HF_CLK_DIV("255")) osc (
		.HFOUTEN(1'b1),
		.HFCLKOUT(clk)
	);

	reg [33:0] ctr = 34'd0;

	always @(posedge clk)
		if (!gsrn)
			ctr <= 34'd0;
		else
			ctr <= ctr + 34'd1;

	assign led = ~ctr[33:20];
endmodule
