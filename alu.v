module ALU(Ain,Bin,ALUop,out,outFlag);
	input [15:0] Ain, Bin;
	input [1:0] ALUop;
	output [15:0] out;
	//zero flag, negative flag, overflow flag
	output [2:0] outFlag;

	reg [15:0] out;
	wire [15:0] postaddsub;
	reg [2:0] outFlag;
	wire ovrflw;
 	
	//add or subtracts the number and detects if there is an overflow
	AddSubOverflow overflow(Ain, Bin, ALUop[0], postaddsub, ovrflw);

	//perform math functions depending on ALUop input
	always @* begin
		//perform math functions
		case (ALUop) 
			2'b00: out = postaddsub;
			2'b01: out = postaddsub;
			2'b10: out = Ain & Bin;
			2'b11: out = ~Bin;
			default: out = 16'bxxxxxxxxxxxxxxxx;
		endcase
		
		//set outFlag[0] to 1 if out is 0s
		if (out == 16'b0000000000000000) 
			outFlag[0] = 1'b1;
		else 
			outFlag[0] = 1'b0;
		
		//set outFlag[1] to 1 if out is negative
		if(out[15] == 1'b1)
			outFlag[1] = 1'b1;
		else
			outFlag[1] = 1'b0;
		
		//set outFlag[2] to 1 if it overflows during an add/sub operation
		if(ovrflw == 1'b1 && (ALUop == 2'b00 || ALUop == 2'b01))
			outFlag[2] = 1'b1;
		else
			outFlag[2] = 1'b0;  	
	end

endmodule

// add a+b or subtract a-b, check for overflow
//code from slide set 6
module AddSubOverflow(a,b,sub,s,ovf) ;
  parameter n = 16;
  input [n-1:0] a, b ;
  input sub ;           // subtract if sub=1, otherwise add
  output [n-1:0] s ;		// final number s[n-1] is sign, rest is value
  output ovf ;          // 1 if overflow
  wire c1, c2 ;         // carry out of last two bits
  wire ovf = c1 ^ c2 ;  // overflow if signs don't match

  // add non sign bits (all bits excluding MSB)
  Adder1 #(n-1) ai(a[n-2:0],b[n-2:0]^{n-1{sub}},sub,c1,s[n-2:0]);
  // add sign bits (the MSB)
  Adder1 #(1)   as(a[n-1],b[n-1]^sub,c1,c2,s[n-1]);
endmodule

// multi-bit adder - behavioral
//code from slide set 6
module Adder1(a,b,cin,cout,s) ;
  parameter n = 8 ;
  input [n-1:0] a, b ;
  input cin ;
  output [n-1:0] s ;
  output cout ;
  wire [n-1:0] s;
  wire cout ;

	//adds the two inputs together with the carry in
  assign {cout, s} = a + b + cin ;
endmodule 

	