module datapath(clk,readnum,vsel,loada,loadb,shift,asel,bsel,ALUop,loadc,loads,writenum,write,mdata,PC,sximm8, sximm5, outFlags,datapath_out);
	
    input clk,asel,bsel,loada,loadb,loadc,loads,write;
    input [1:0] shift,ALUop;
    input [2:0] writenum,readnum;
	input [3:0] vsel;
	input [8:0] PC;
    input [15:0] mdata, sximm8, sximm5;

    output [2:0] outFlags;
    output [15:0] datapath_out;

    wire [15:0] data_in,data_out;
    wire [15:0] loadaout,loadbout,sout;
    wire [15:0] Ain,Bin,out;
	wire [2:0] outFlag;
    wire [15:0] datapath_out;

	//4 input multiplexer that assigns the value sent to register depending on vsel
	//C -> 0 (0001), mdata -> 4 (1000)
	Mux4in vselOut(datapath_out, {7'b0, PC}, sximm8, mdata, vsel, data_in);

	//register file that performs the writing and reading
    regfile REGFILE(data_in, writenum, write, readnum, clk, data_out);

	//store specific outputs from the register
    loadEnableRegister LOADAA(clk, loada, data_out, loadaout);
    loadEnableRegister LOADBB(clk, loadb, data_out, loadbout);

	//multiplexer assigns depending on asel
    assign Ain = asel ? 16'b0 : loadaout;

	//shifts (multiply / divide) output from B depending on shift input value
    shifter SHIFT(loadbout,shift,sout);
	//multiplexer assigns depending on bsel
    assign Bin = bsel ? sximm5 : sout;

	//performs math functions on Ain and Bin depending on ALUop input value
	ALU ALU(Ain,Bin,ALUop,out, outFlag);

	//stores the outputs from ALU
	loadEnableRegister LOADCC(clk, loadc, out, datapath_out);
	//status register is 3 bits for the different flags
	loadEnableRegister #(3) LOADSS(clk, loads, outFlag, outFlags);


endmodule
