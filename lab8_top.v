`define MREAD 2'b01;
`define MWRITE 2'b11;
`define MNONE 2'b00;

module lab8_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,CLOCK_50);
	input [3:0] KEY;
	input [9:0] SW;
	input CLOCK_50;
	output [9:0] LEDR;
	output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

	wire clk = CLOCK_50;
	wire reset = ~KEY[1];
	wire N,V,Z;
	wire mWriteEq, mReadEq, msel;
	wire [15:0] dout;
    wire write, read;
    wire [2:0] outFlags;
    wire [15:0] write_data;
    wire [15:0] read_data;
    wire [1:0] mem_cmd;
    wire [8:0] mem_addr;
    wire outRead, outWrite;

	//send data to the cpu to obtain commands and addresses
	//need to replace clk with one of the KEYs !!! same with reset
	cpu CPU(clk,reset,read_data,write_data,N,V,Z, mem_addr,mem_cmd);

	//RAM instantiation (place values or take values with RAM)
	RAM #(16,8) MEM(clk,mem_addr[7:0],mem_addr[7:0],write,write_data,dout);

	//equality comparators that checks if the wires should go through
	EqComp #(2) mwrite(2'b11, mem_cmd, mWriteEq);
	EqComp #(2) mread(2'b01, mem_cmd, mReadEq);
	EqComp #(1) msel0(mem_addr[8], 1'b0, msel);

	//AND the wires together
	assign write = mWriteEq & msel;
	assign read = mReadEq & msel;

	//tri state driver
	assign read_data = read ? dout : 16'bz;

	//combination logic that checks the values from mem_addr and mem_cmd and outputs 1 if they fit the criteria
	combLogic read0(9'h140, mem_addr, mem_cmd, 2'b01, outRead);
	combLogic write0(9'h100, mem_addr, mem_cmd, 2'b11, outWrite);

	//if outRead is 1 that will set read_data to SW and if outWrite is 1 will set LEDs to write_data
	assign read_data[7:0] = outRead ? SW : 8'bz;
	assign read_data[15:8] = outRead ? 8'h00 : 8'bz;
	loadEnableRegister #(8) writeData(clk, outWrite, write_data[7:0], LEDR[7:0]);


endmodule

//RAM holds different binary values in memory locations (specific set size)
module RAM(clk,read_address,write_address,write,din,dout);
	parameter data_width = 32; 
	parameter addr_width = 4;
	parameter filename = "data.txt";

	input clk;
	input [addr_width-1:0] read_address, write_address;
	input write;
	input [data_width-1:0] din;
	output [data_width-1:0] dout;
	reg [data_width-1:0] dout;

	reg [data_width-1:0] mem [2**addr_width-1:0];

	initial $readmemb(filename, mem);

	always @ (posedge clk) begin
	if (write)
		mem[write_address] <= din;
	dout <= mem[read_address]; // dout doesn't get din in this clock cycle (this is due to Verilog non-blocking assignment "<=")
	end 
endmodule

//combination logic block that decides if output is 1 depending on if mem_addr = val and mem_cmd is same is required input
module combLogic(val, mem_addr, mem_cmd1, mem_cmd2, out);
	input [8:0] val, mem_addr;
	input [1:0] mem_cmd1;
	input [1:0] mem_cmd2;
	output out;

	wire imm, imm2;

	assign imm = (val == mem_addr);
	assign imm2 = (mem_cmd1 == mem_cmd2);

	assign out = imm & imm2; 
endmodule

// equality comparator
module EqComp(a, b, eq) ;
  parameter k=8;
  input  [k-1:0] a,b;
  output eq;
  wire   eq;

  assign eq = (a==b) ;
endmodule
