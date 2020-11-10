`define SRST 4'b0000
`define DECODE 4'b1000
`define Sa  4'b0001
`define Sb  4'b0010
`define Sc  4'b0011
`define Sd  4'b0100
`define Se  4'b1001
`define Sf  4'b1010
`define Sg  4'b1011
`define Sh  4'b1100
`define SIF1 4'b0101
`define SIF2 4'b0110
`define Supdate 4'b0111
`define Shalt 4'b1111
`define Su  4'bxxxx

`define MNONE 2'b00
`define MREAD 2'b01 
`define MWRITE 2'b11

//CPU logic block
module cpu(clk,reset,read_data,out,N,V,Z,mem_addr,mem_cmd);
	input clk, reset;
	input [15:0] read_data;
	output [15:0] out;
	output N, V, Z;
    output [1:0] mem_cmd;
    output [8:0] mem_addr;

	wire[15:0] instr_out;
    wire [2:0] opcode, writenum, readnum, out_mux; 
    wire [1:0] opType;
    wire [2:0] Rn,Rd,Rm;
    wire [1:0] shift;
    wire [15:0] sximm8;
    wire [15:0] sximm5;
    wire w; //is 1 when reset
    wire [3:0] vsel; //one hot select
    wire [2:0] nsel; //nsel is a one-hot select, 001 for Rn, 010 for Rd, and 100 for Rm
    wire loada,loadb,loadc,loads;
    wire asel,bsel;
    wire write;
    wire [2:0] outFlags;
    wire [15:0] datapath_out;
    wire [15:0] mdata = read_data;
    wire [7:0] PCwire = 8'b0;
    wire addr_sel, load_ir, load_pc, reset_pc, load_addr;
    wire [8:0] next_pc, PC, data_addr_out;


	//when clock is 1 and load is 1 store 'in' in register
	loadEnableRegister instructionReg(clk, load_ir, read_data, instr_out);

    //decode all the instructions so we have opcode, op to send to fsm and rest to datapath
    InstruDecode instr1(instr_out,opcode,opType,Rn,Rd,shift,Rm,sximm8,sximm5);
	
    Mux3in MUX1(Rn,Rd,Rm, nsel, out_mux);

    assign readnum = out_mux;
    assign writenum = out_mux;

    //finite state machine that chooses values to be sent to the datapath based on instructions given to it
    fsm FSM1(clk,reset,opcode,opType,nsel,vsel,loada,loadb,asel,bsel,loadc,loads,write,mem_cmd,addr_sel,load_ir,load_pc,reset_pc,load_addr);

    //multiplexer that determines the input to the program counter
    assign next_pc = reset_pc ? 9'b0 : PC + 1'b1;

    //PROGRAM COUNTER
    loadEnableRegister #(9) PC0(clk, load_pc, next_pc, PC);

    //DATA ADDRESS
    loadEnableRegister #(9) DataAddress(clk, load_addr, datapath_out[8:0], data_addr_out);

    //multiplexer that determines the output to the mem_addr
    assign mem_addr = addr_sel ? PC : data_addr_out;

    //datapath that does all the calculations and stores the values for the registers
    datapath DP(clk,readnum,vsel,loada,loadb,shift,asel,bsel,opType,loadc,loads,writenum,write,mdata,PCwire,sximm8, sximm5, outFlags,datapath_out);

    //assign final outputs, N -> neg, V  -> overflow, Z -> zero, out = final calculation
    assign out = datapath_out;
    assign N = outFlags[1];
    assign V = outFlags[2];
    assign Z = outFlags[0];
endmodule

module fsm(clk,reset,opcode,op,nsel,vsel,loada,loadb,asel,bsel,loadc,loads,write, mem_cmd, addr_sel, load_ir, load_pc, reset_pc, load_addr);
	input clk,reset;
	input [2:0] opcode;
	input [1:0] op;
	output [3:0] vsel; //one hot select
	output [2:0] nsel; //nsel is a one-hot select, 100 for Rn, 010 for Rd, and 001 for Rm
	output loada,loadb,loadc,loads;
	output asel,bsel;
	output write;
    output [1:0] mem_cmd;
    output addr_sel, load_ir, load_pc, reset_pc, load_addr;


	reg [3:0] present_state; //refer to lab6 powerpoint slide 10
	reg [2:0] nsel;
	reg loada,loadb,loadc,loads;
	reg asel,bsel;
	reg [3:0] vsel; //one hot
	reg write;
	reg w;

    reg [1:0] mem_cmd;
    reg addr_sel, load_ir, load_pc, reset_pc, load_addr;

	always @(posedge clk) begin
		if (reset) begin
			present_state = `SRST;
            reset_pc = 1'b1;
            load_pc = 1'b1;
            write = 1'b0;
            loadc = 1'b0;
            loads = 1'b0;
            load_ir = 1'b0;
            addr_sel = 1'b0;
            mem_cmd = `MNONE;
            load_addr = 1'b0;
		end else 
        begin
			casex ({opcode, present_state})

                {3'bxxx, `SRST}:
                begin
                    load_pc = 1'b0;
                    present_state = `SIF1;
                    reset_pc = 1'b0;
                    addr_sel = 1'b1;
                    mem_cmd = `MREAD;
                    load_addr = 1'b0;
                end

                {3'bxxx, `SIF1}:
                begin
                    load_pc = 1'b0;
                    present_state = `SIF2;
                    addr_sel = 1'b1;
                    load_ir = 1'b1;
                    mem_cmd = `MREAD;
                end

                {3'bxxx, `SIF2}:
                begin
                    present_state = `Supdate;
                    load_pc = 1'b1;
                    load_ir = 1'b0;
                    addr_sel = 1'b0;
                    mem_cmd = `MNONE;
                end


    			//MOVE FUNCTIONS
    			{3'b110, 4'bxxxx}:
                begin
                    casex (op)
                        //MOVE EXTENDED -> RN
                        2'b10:
                        begin
                            casex (present_state)
                                `Supdate: present_state = `DECODE;
                                `DECODE: present_state = `Sa;
                                `Sa: present_state = `SIF1;
                                default: present_state = `Su;
                            endcase

                            casex (present_state)
                                `DECODE:
                                begin
                                    load_pc = 1'b0;
                                end
                                `Sa:
                                begin
                                    vsel = 4'b0100;
                                    nsel = 3'b001;
                                    write = 1'b1;
                                end
                                `SIF1: 
                                begin
                                    addr_sel = 1'b1;
                                    mem_cmd = `MREAD;
                                    reset_pc = 1'b0;
                                    write = 1'b0;
                                    load_addr = 1'b0;
                                    load_pc = 1'b0;
                                    loadc = 1'b0;
                                    loads = 1'b0;
                                end
                                default: nsel = `Su;
                            endcase
                        end
                        //MOVE RM -> RN
                        2'b00:
                        begin
                            casex (present_state)
                                `Supdate: present_state = `DECODE;
                                `DECODE: present_state = `Sa;
                                `Sa: present_state = `Sb;
                                `Sb: present_state = `Sc;
                                `Sc: present_state = `SIF1;
                                default: present_state = `Su;
                            endcase

                            casex (present_state)
                                `DECODE:
                                begin
                                    load_pc = 1'b0;
                                end
                                `Sa:
                                begin
                                     nsel = 3'b100;
                                     loada = 1'b0;
                                     loadb = 1'b1;
                                     write = 1'b0;
                                end
                                `Sb:
                                begin
                                     asel = 1'b1;
                                     bsel = 1'b0;
                                     loadc = 1'b1;
                                     write = 1'b0;
                                end
                                `Sc:
                                begin
                                    nsel = 3'b010;
                                    vsel = 4'b0001; //C is binary value 00 for vsel decoder in datapath
                                    write = 1'b1;
                                end
                                `SIF1:
                                begin
                                    addr_sel = 1'b1;
                                    mem_cmd = `MREAD;
                                    reset_pc = 1'b0;
                                    load_addr = 1'b0;
                                    load_pc = 1'b0;
                                    write = 1'b0;
                                    loadc = 1'b0;
                                    loads = 1'b0;
                                end
                                default: nsel = `Su;
                            endcase
                        end
                        default: present_state = `Su;
                    endcase
                end

    			//ALU FUNCTIONS
                {3'b101, 4'bxxxx}:
    			begin
                    casex (present_state)
                        `Supdate: present_state = `DECODE;
                        `DECODE: present_state = `Sa;
                        `Sa: present_state = `Sb;
                        `Sb: present_state = `Sc;
                        `Sc: present_state = `Sd;
                        `Sd: present_state = `SIF1;
                        default: present_state = `Su;
                    endcase
                    casex (present_state)
                        `DECODE:
                        begin
                            load_pc = 1'b0;
                        end
                        `Sa:
                        begin
                            nsel = 3'b001;
                            loada = 1'b1;
                            loadb = 1'b0;
                            write = 1'b0;
                        end
                        `Sb:
                        begin
                            nsel = 3'b100;
                            loadb = 1'b1;
                            loada = 1'b0;
                            write = 1'b0;
                        end
                       `Sc:
                        begin
                            asel = 1'b0;
                            bsel = 1'b0;
                            if (op == 2'b01) begin
                                loadc = 1'b0;
                                loads = 1'b1;
                            end else begin
                                loadc = 1'b1;
                                loads = 1'b0;
                            end 
                            write = 1'b0;
                        end
                        `Sd:
                        begin
                            nsel = 3'b010;
                            vsel = 4'b0001; //C is binary value 00 for vsel decoder in datapath
                            w = 1'b0;
                            if (op == 2'b01)
                                write = 1'b0;
                            else
                                write = 1'b1;
                        end
                        `SIF1:
                        begin
                            addr_sel = 1'b1;
                            mem_cmd = `MREAD;
                            reset_pc = 1'b0;
                            load_addr = 1'b0;
                            load_pc = 1'b0;
                            write = 1'b0;
                            loadc = 1'b0;
                            loads = 1'b0;
                        end
                        default: nsel = `Su;
                    endcase
                end

                //LDR 
                {3'b011, 4'bxxxx}:
                begin
                    casex (present_state)
                        `Supdate: present_state = `DECODE;
                        `DECODE: present_state = `Sa;
                        `Sa: present_state = `Sb;
                        `Sb: present_state = `Sc;
                        `Sc: present_state = `Sd;
                        `Sd: present_state = `Se;
                        `Se: present_state = `Sf;
                        `Sf: present_state = `SIF1;
                        default: present_state = `Su;
                    endcase
                    casex (present_state)
                        `DECODE:
                        begin
                            load_pc = 1'b0;
                        end
                        `Sa:
                        begin
                            nsel = 3'b001;
                            loada = 1'b1;
                            loadb = 1'b0;
                            write = 1'b0;
                        end
                        `Sb:
                        begin
                            nsel = 3'b100;
                            loadb = 1'b1;
                            loada = 1'b0;
                            write = 1'b0;
                        end
                        `Sc:
                        begin
                            asel = 1'b0;
                            bsel = 1'b1;
                            loadc = 1'b1;
                            loads = 1'b0;
                            write = 1'b0;
                        end
                        `Sd:
                        begin
                            load_addr = 1'b1;
                        end
                        `Se:
                        begin
                            addr_sel = 1'b0;
                            load_addr = 1'b0;
                            mem_cmd = `MREAD;
                        end
                        `Sf:
                        begin
                            nsel = 3'b010;
                            vsel = 4'b1000; //Mdata is the value read from memory
                            write = 1'b1;
                        end
                        `SIF1:
                        begin
                            addr_sel = 1'b1;
                            mem_cmd = `MREAD;
                            reset_pc = 1'b0;
                            load_addr = 1'b0;
                            load_pc = 1'b0;
                            write = 1'b0;
                            loadc = 1'b0;
                            loads = 1'b0;
                        end
                        default: nsel = `Su;
                    endcase
                end

                //STR !!!!probably not complete!!!!!
                {3'b100, 4'bxxxx}:
                begin
                    casex (present_state)
                        `Supdate: present_state = `DECODE;
                        `DECODE: present_state = `Sa;
                        `Sa: present_state = `Sb;
                        `Sb: present_state = `Sc;
                        `Sc: present_state = `Sd;
                        `Sd: present_state = `Se;
                        `Se: present_state = `Sf;
                        `Sf: present_state = `Sg;
                        `Sg: present_state = `Sh;
                        `Sh: present_state = `SIF1;
                        default: present_state = `Su;
                    endcase
                    casex (present_state)
                        `DECODE:
                        begin
                            load_pc = 1'b0;
                        end
                        `Sa:
                        begin
                            nsel = 3'b001;
                            loada = 1'b1;
                            loadb = 1'b0;
                            write = 1'b0;
                        end
                        `Sb:
                        begin
                            nsel = 3'b100;
                            loadb = 1'b1;
                            loada = 1'b0;
                            write = 1'b0;
                        end
                        `Sc:
                        begin
                            asel = 1'b0;
                            bsel = 1'b1;
                            loadc = 1'b1;
                            loads = 1'b0;
                            write = 1'b0;
                        end
                        `Sd:
                        begin
                            load_addr = 1'b1;
                        end
                        `Se:
                        begin
                            addr_sel = 1'b0;
                            load_addr = 1'b0;
                            mem_cmd = `MNONE;
                        end
                        `Sf:
                        begin
                            nsel = 3'b010;
                            vsel = 4'b1000; //Mdata is the value read from memory
                            write = 1'b0;
                            loada = 1'b0;
                            loadb = 1'b1;
                        end
                        `Sg:
                        begin
                            asel = 1'b1;
                            bsel = 1'b0;
                            loadc = 1'b1;
                            loads = 1'b0;
                        end
                        `Sh:
                        begin
                            addr_sel = 1'b0;
                            mem_cmd = `MWRITE;
                        end
                        `SIF1:
                        begin
                            addr_sel = 1'b1;
                            mem_cmd = `MREAD;
                            reset_pc = 1'b0;
                            load_addr = 1'b0;
                            load_pc = 1'b0;
                            write = 1'b0;
                            loadc = 1'b0;
                            loads = 1'b0;
                        end
                        default: nsel = `Su;
                    endcase
                end

                //HALT
                {3'b111, 4'bxxxx}: present_state = `Shalt;
                {3'bxxx, `Shalt}: present_state = `Shalt;
                default: present_state = `Su;
            endcase
    	end
    end
endmodule


module InstruDecode(instruction,opcode,opType,Rn,Rd,shift,Rm,sximm8,sximm5);
	input [15:0] instruction; //16 bit instruction 
	output [2:0] opcode; 
	output [1:0] opType;
	output [2:0] Rn,Rd,Rm;
	output [1:0] shift;
	output [15:0] sximm8;
	output [15:0] sximm5;

	wire [7:0] imm8;
	wire [4:0] imm5;

	//the following assign statements are based off the suggested chart
	assign opcode = instruction[15:13];
	assign opType = instruction[12:11]; 
	assign Rn = instruction[10:8];
	assign Rd = instruction[7:5];
	assign Rm = instruction[2:0];
	assign shift = instruction[4:3];
	assign imm8 = instruction[7:0];
	assign imm5 = instruction[4:0];

	//sign extend imm5 and imm8 to be 16 bits
	SignExtend #(5) Extend5to16(imm5, sximm5);
	SignExtend #(8) Extend8to16(imm8, sximm8);

endmodule

//module for sign extending a signal with n bits
module SignExtend(in,out); 
	parameter n;

	input [n-1:0] in;
	output [15:0] out;
	reg [15:0] out;

	always @(*)
		case(in[n-1])
			1'b0: out = {{16-n{1'b0}},in};
			1'b1: out = {{16-n{1'b1}},in};
			default: out = 16'bxxxxxxxxxxxxxxxx;
		endcase
endmodule