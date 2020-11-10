module regfile(data_in, writenum, write, readnum, clk, data_out);
	input [15:0] data_in;
	input [2:0] writenum, readnum;
	input write, clk;
	output[15:0] data_out;

	wire decoder0, decoder1, decoder2, decoder3, decoder4, decoder5, decoder6, decoder7;
	wire[15:0] R0, R1, R2, R3, R4, R5, R6, R7,final;
	wire[7:0] oneHot,oneHot2;

	// turn both writenum and readnum to one-hot from binary
	Dec D1(writenum, oneHot);
	Dec D2(readnum, oneHot2);

	// if both oneHot and write values are 1 then send the signal to specific register
	assign decode0 = oneHot[0] & write;
	assign decode1 = oneHot[1] & write;
	assign decode2 = oneHot[2] & write;
	assign decode3 = oneHot[3] & write;
	assign decode4 = oneHot[4] & write;
	assign decode5 = oneHot[5] & write;
	assign decode6 = oneHot[6] & write;
	assign decode7 = oneHot[7] & write;

	// store data depending on which 
	loadEnableRegister REG0(clk, decode0, data_in, R0);
	loadEnableRegister REG1(clk, decode1, data_in, R1);
	loadEnableRegister REG2(clk, decode2, data_in, R2);
	loadEnableRegister REG3(clk, decode3, data_in, R3);
	loadEnableRegister REG4(clk, decode4, data_in, R4);
	loadEnableRegister REG5(clk, decode5, data_in, R5);
	loadEnableRegister REG6(clk, decode6, data_in, R6);
	loadEnableRegister REG7(clk, decode7, data_in, R7);

	//choose which register data should be output depending on readnum one-hot value
	Mux8in regOut(R0, R1, R2, R3, R4, R5, R6, R7, oneHot2, final);

	assign data_out = final;
endmodule

//decoder that turns a 3 bit binary value into a 8 bit one-hot code
module Dec(a, b);
	parameter n = 3;
	parameter m = 8;

	input [n-1:0] a;
	output [m-1:0] b;

	wire [m-1:0] b = 1 << a;
endmodule

//register with load enable for storing 16 bit values
module loadEnableRegister(clk, load, data_in, out);
	parameter n = 16;
  input clk, load;
  input  [n-1:0] data_in;
  output [n-1:0] out;
  reg    [n-1:0] out;
  wire   [n-1:0] next_out;

  assign next_out = load ? data_in : out;

  always @(posedge clk)
    out = next_out;  
endmodule 

//multiplexer for 8, 16 bit inputs
module Mux8in(R0, R1, R2, R3, R4, R5, R6, R7, D, out) ;
  parameter k = 16;
	input [k - 1:0] R0, R1, R2, R3, R4, R5, R6, R7;
  input [7:0] D;
  output [k - 1:0] out ;

	//copies the D value with each register input
  assign out = ({k{D[0]}} & R0) |
				({k{D[1]}} & R1) |
				({k{D[2]}} & R2) |
				({k{D[3]}} & R3) |
				({k{D[4]}} & R4) |
				({k{D[5]}} & R5) |
				({k{D[6]}} & R6) |
				({k{D[7]}} & R7);
endmodule

//multiplexer for 4, 16 bit inputs
module Mux4in(in0, in1, in2, in3, D, out) ;
  parameter k = 16;
  input [k - 1:0] in0, in1, in2, in3;
  input [3:0] D;
  output [k - 1:0] out ;

	//copies the D value with each input
  assign out = ({k{D[0]}} & in0) |
				({k{D[1]}} & in1) |
				({k{D[2]}} & in2) |
				({k{D[3]}} & in3);
endmodule 

module Mux3in(in0, in1, in2, D, out);
  parameter k = 3;
  input [k - 1:0] in0, in1, in2;
  input [2:0] D;
  output [k - 1:0] out;

	//copies the D value with each input
  assign out = ({k{D[0]}} & in0) |
				({k{D[1]}} & in1) |
				({k{D[2]}} & in2);
endmodule 