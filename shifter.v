module shifter(in,shift,sout);
	input [15:0] in;
	input [1:0] shift;
	output [15:0] sout;

	reg [15:0] sout;

	//perform shifts depending on shift value
	always @* begin
		case (shift) 
			2'b00: sout = in;
			2'b01: sout = in << 1;
			2'b10: sout = in >> 1;
			//shift right and copy value from bit 15 -> 16
			2'b11: sout = {in[15], in[15], in[14], in[13], in[12], in[11], in[10], in[9], 
											in[8], in[7], in[6], in[5], in[4], in[3], in[2], in[1]};
			default: sout = 16'bxxxxxxxxxxxxxxxx;
		endcase
	end
endmodule 