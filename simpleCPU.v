module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );

	
	// wires
	//==============================================================================
	
	// Instruction wires
	//------------------------------------
	wire [4:0]  Rs;
	wire [4:0]  Rt;
	wire [4:0]  Rd;
	wire [5:0]  Opcode;
	wire [5:0]  Funct;
	wire [4:0]  Shamt;
	wire [15:0] Immediate;
	wire [31:0] SignImm;
	//------------------------------------

	// ControlUnit wires
	//------------------------------------
	wire [3:0] ALUControl;
	wire RegWrite, RegDst, ALUSrc, Branch, MemWrite, MemToReg, Jal, Jr;
	//------------------------------------

	// Register wires
	//------------------------------------
	wire[4:0]  A1Input, A2Input, A3Input;
	wire[31:0] WD3Input;
	wire[31:0] RD1Wire, RD2Wire;
	wire[4:0]  WriteReg;
	//------------------------------------

	// ALU wires
	//------------------------------------
	wire[31:0] SrcA, SrcB, ALUOut;
	wire Zero;
	//------------------------------------

	// PC wires
	//------------------------------------
	wire[31:0] inputPC, outputPC, PCPlus4, PCBranch, PCJal, PCJr;
	wire beqWire;
	//------------------------------------

	// Memory/ALU wire output
	//------------------------------------
	wire[31:0] Result;
	//------------------------------------

	// assignings
	//==============================================================================

	assign Opcode =    instruction[31:26];
	assign Rs =        instruction[25:21];
	assign Rt =        instruction[20:16];
	assign Rd =        instruction[15:11];
	assign Shamt =     instruction[10:6];
	assign Funct =     instruction[5:0];
	assign Immediate = instruction[15:0];
	Extend16_32 myExtend(Immediate, SignImm);


	ControlUnit myControlUnit(Opcode, Funct, Shamt,
		                        ALUControl,
		                        RegWrite, RegDst, ALUSrc, Branch, MemWrite, MemToReg, Jal, Jr);

	assign A1Input = Rs;
	assign A2Input = Rt;
	Multx5b_2in MLTXJal_RegisterA3(5'b11111, WriteReg, Jal, A3Input); // assigning A3Input
	Multx5b_2in MLTXRegDst_RtOrRd(Rd, Rt, RegDst, WriteReg);    // assigning WriteReg
	Multx32b_2in MLTXJal_RegisterWD3(PCPlus4, Result, Jal, WD3Input); // assigning A3Input

	Register32_Array myRegArray (RegWrite, clk, A1Input, A2Input, A3Input,
		                           WD3Input,
		                           RD1Wire, RD2Wire);

	assign SrcA = RD1Wire;
	Multx32b_2in MLTXAluSrc_ALU(SignImm, RD2Wire, ALUSrc, SrcB);
	ALU2 myAlu (SrcA, SrcB, ALUControl,
		          Zero, ALUOut);	
	Multx32b_2in MLTXMemToReg_CPUResult(data_from_mem, ALUOut, MemToReg, Result);

	assign PCPlus4 = outputPC + 4;
	assign PCBranch = (SignImm << 2) + (PCPlus4);
	assign PCJr = RD1Wire;
	assign PCJal = { PCPlus4[31:28], instruction[25:0], 2'b00 };

	assign beqWire = (Branch & Zero);	
	PCMutex myPCMultx(PCJr, PCJal, PCBranch, PCPlus4,
                    Jr, Jal, beqWire,
                    inputPC);
	
	Register32_reset myPC(inputPC, clk, reset, outputPC);
	assign PC = outputPC;

	assign WE =             MemWrite;
	assign data_to_mem =    RD2Wire;
	assign address_to_mem = ALUOut;
/*
	always @(clk) begin  $display( "Opcod= %b, Reg1= %d, SrcA= %d, Reg2= %d, Imm= %d, SrcB= %d, AluRes= %d, AluZer= %d, AluControl= %b, MemRes= %d, PC=%d",
		                              Opcode, RD1Wire, SrcA, RD2Wire, SignImm, SrcB, ALUOut, Zero, ALUControl, Result, (PC/4)+1); end
*/

endmodule

//=====================================================================
module Multx5b_2in(input[4:0] a, b, input s, output[4:0] out);
	assign out = s==1 ? a : b;
endmodule
//=====================================================================
module Multx32b_2in(input[31:0] a, b, input s, output[31:0] out);
	assign out = s==1 ? a : b;
endmodule
//=====================================================================
module Comparator(input [31:0] ina, inb, output out);
	assign out = ina==inb;
endmodule
//=====================================================================
module Extend16_32(input [15:0] in, output [31:0] out);
	assign out = {{16{in[15]}},in[15:0]};
endmodule
//=====================================================================
module Register32(input[31:0] in,  input clk, output reg [31:0] out);
	always @(posedge clk) begin 
	    out = in;
	end
endmodule
//=====================================================================
module Register32_reset( input[31:0] in,
	                     input clk, reset,
	                     output reg [31:0] out);
	always @(posedge clk or posedge reset) begin
		if (reset)	out = 0;
		else        out = in;
	end
endmodule
//=====================================================================
module Register32_Array( input WE3, clk, input[4:0] A1, A2, A3,
                         input[31:0] WD3,
                         output [31:0] RD1, RD2);
	reg [31:0] reg_array [31:0];
	assign RD1 = (A1==0) ? 0 : reg_array[A1[4:0]];
	assign RD2 = (A2==0) ? 0 : reg_array[A2[4:0]];
	always @(posedge clk) begin	
		if (WE3) begin
			reg_array[A3[4:0]] = WD3[31:0];
		end
	end
endmodule
//=====================================================================
module PCMutex( input[31:0] jr, jal, beq, plus4,
								input sJr, sJal, sBeq,
								output reg[31:0] pc);
always @ (*) begin
	if      (sJr == 1)	pc = jr;
	else if (sJal ==1)	pc = jal;
	else if (sBeq == 1)	pc = beq;
	else                pc = plus4;
end
endmodule
//=====================================================================
module ALU2 (
input[31:0] SrcA, SrcB, 
input[3:0]  ALUControl, 
output reg  Zero,
output reg[31:0] ALUResult
);

wire [33:0] w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11;
// structure of every wire: w[32] = CARRY, w[33] = OVERFLOW, w[31:0] = ALURESULT

complementAdd32B                myCircuit1(SrcA, SrcB, w1[31:0], w1[32], w1[33]);   // add
complementSub32B                myCircuit2(SrcA, SrcB, w2[31:0], w2[32], w2[33]);   // sub
logicAND32B                     myCircuit3(SrcA, SrcB, w3[31:0]);                   // and
logicOR32B                      myCircuit4(SrcA, SrcB, w4[31:0]);                   // or
logicXOR32B                     myCircuit5(SrcA, SrcB, w5[31:0]);                   // xor
slt32B                          myCircuit6(SrcA, SrcB, w6[31:0]);                   // slt
FourTimesUnsignedAdd8B          myCircuit7(SrcA, SrcB, w7[31:0]);                   // 4 unsigned sums
FourTimesSaturatedUnsignedAdd8B myCircuit8(SrcA, SrcB, w8[31:0]);                   // 4 unsigned saturated sums
srlv32B                         myCircuit9(SrcA, SrcB, w9[31:0]);                   // srlv
sllv32B                        myCircuit10(SrcA, SrcB, w10[31:0]);                  // sllv
srav32B                        myCircuit11(SrcA, SrcB, w11[31:0]);                  // srav

always @ (*) begin
	case (ALUControl)
	  4'b0010 : begin ALUResult = w1[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // add
	  4'b0110 : begin ALUResult = w2[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // sub
	  4'b0000 : begin ALUResult = w3[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // and
	  4'b0001 : begin ALUResult = w4[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // or
	  4'b0011 : begin ALUResult = w5[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // xor
	  4'b0111 : begin ALUResult = w6[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // slt
	  4'b1000 : begin ALUResult = w7[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // 4 unsigned sums
	  4'b1001 : begin ALUResult = w8[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // 4 unsigned saturated sums
	  4'b0101 : begin ALUResult = w9[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // srlv
	  4'b1010 : begin ALUResult = w10[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // sllv
	  4'b1111 : begin ALUResult = w11[31:0]; Zero = (ALUResult == 0) ? 1 : 0; end // srav
	endcase
end
endmodule
//=====================================================================
//----------------------------------------------------------
module complementAdd32B(
	input [31:0] a,
	input [31:0] b,
    output [31:0] sum,
	output carry,
	output overflow);
	
	wire signA;
	wire signB;
	wire signSum;
	wire [32:0] widenSum;

	assign widenSum = a + b;

	assign sum = widenSum[31:0];
	assign signSum = sum[31];
	assign signA = a[31];
	assign signB = b[31];

	assign carry = widenSum[32];
	assign overflow = ((signA == signB) && (signA != signSum )) ? 1'd1 : 1'd0;
endmodule
//----------------------------------------------------------
module FourTimesUnsignedAdd8B(
	input [31:0] a,
	input [31:0] b,
    output [31:0] sum);
	
	wire [7:0] a1, a2, a3, a4;
	wire [7:0] b1, b2, b3, b4;
	
	assign a1 = a[7:0]; assign a2 = a[15:8]; assign a3 = a[23:16]; assign a4 = a[31:24];
	assign b1 = b[7:0]; assign b2 = b[15:8]; assign b3 = b[23:16]; assign b4 = b[31:24];

	assign sum = {a4+b4, a3+b3, a2+b2, a1+b1};
endmodule
//----------------------------------------------------------
module SaturatedUnsignedAdd8B(
	input [7:0] a,
	input [7:0] b,
	output [7:0] sum);

	wire [8:0] widenSum;
	wire [8:0] widenA;
	wire [8:0] widenB;
	assign widenA = {1'b0, a[7:0]};
	assign widenB = {1'b0, b[7:0]};


	assign widenSum = widenA + widenB;
	
	assign sum = (widenSum[8] == 1) ? 8'b11111111 : a + b;
endmodule
//----------------------------------------------------------
module FourTimesSaturatedUnsignedAdd8B(
	input [31:0] a,
	input [31:0] b,
	output [31:0] sum);

	SaturatedUnsignedAdd8B myCircuit1(a[7:0],   b[7:0],   sum[7:0]);
	SaturatedUnsignedAdd8B myCircuit2(a[15:8],  b[15:8],  sum[15:8]);
	SaturatedUnsignedAdd8B myCircuit3(a[23:16], b[23:16], sum[23:16]);
	SaturatedUnsignedAdd8B myCircuit4(a[31:24], b[31:24], sum[31:24]);
endmodule
//----------------------------------------------------------
module complementSub32B(
	input [31:0] a,
	input [31:0] b,
  output [31:0] sum,
	output carry,
	output overflow);

	wire[31:0] complementB;
	assign complementB = (~b) + 1;
	complementAdd32B add(a, complementB, sum, carry, overflow);
endmodule
//----------------------------------------------------------
module srlv32B(
	input [31:0] a,
	input [31:0] b, // is unsigned
	output [31:0] result);

	assign result = b >> a;
	// always @(result) begin  $display( "SRLV: a= %d, b= %d, result= %b", a, b, (b >> a)); end
endmodule
//----------------------------------------------------------
module sllv32B(
	input [31:0] a,
	input [31:0] b,
	output [31:0] result);

	assign result = b << a;
endmodule
//----------------------------------------------------------
module srav32B(
	input [31:0] a,
	input signed [31:0] b, // is signed
	output [31:0] result);

	assign result = b >>> a;
endmodule
//----------------------------------------------------------
module logicAND32B(
	input [31:0] a,
	input [31:0] b,
	output [31:0] result);

	assign result = a & b;
endmodule
//----------------------------------------------------------
module logicOR32B(
	input [31:0] a,
	input [31:0] b,
	output [31:0] result);

	assign result = a | b;
endmodule
//----------------------------------------------------------
module logicXOR32B(
	input [31:0] a,
	input [31:0] b,
	output [31:0] result);

	assign result = a ^ b;
endmodule
//----------------------------------------------------------
module slt32B(
	input [31:0] a,
	input [31:0] b,
	output [31:0] result);

	assign result = (a < b) ? 1 : 0;
endmodule
//----------------------------------------------------------
//-----------------------------------------------------------
module MainDecoder(
input[5:0] Opcode,
output reg RegWrite, RegDst, ALUSrc,
output reg[1:0] ALUOp,
output reg  Branch, MemWrite, MemToReg, Jal, Jr);
always @ (*) begin
	case (Opcode)
	  6'b000000 : {RegWrite, RegDst, ALUSrc, ALUOp, Branch, MemWrite, MemToReg, Jal, Jr} = {1'b1, 1'b1, 1'b0, 2'b10, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};
	  6'b100011 : {RegWrite, RegDst, ALUSrc, ALUOp, Branch, MemWrite, MemToReg, Jal, Jr} = {1'b1, 1'b0, 1'b1, 2'b00, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0};
	  6'b101011 : {RegWrite, RegDst, ALUSrc, ALUOp, Branch, MemWrite, MemToReg, Jal, Jr} = {1'b0, 1'b0, 1'b1, 2'b00, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0};
	  6'b000100 : {RegWrite, RegDst, ALUSrc, ALUOp, Branch, MemWrite, MemToReg, Jal, Jr} = {1'b0, 1'b0, 1'b0, 2'b01, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0};
	  6'b001000 : {RegWrite, RegDst, ALUSrc, ALUOp, Branch, MemWrite, MemToReg, Jal, Jr} = {1'b1, 1'b0, 1'b1, 2'b00, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};
	  6'b000011 : {RegWrite, RegDst, ALUSrc, ALUOp, Branch, MemWrite, MemToReg, Jal, Jr} = {1'b1, 1'b0, 1'b0, 2'b00, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0};
	  6'b000111 : {RegWrite, RegDst, ALUSrc, ALUOp, Branch, MemWrite, MemToReg, Jal, Jr} = {1'b0, 1'b0, 1'b0, 2'b00, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1};
	  6'b011111 : {RegWrite, RegDst, ALUSrc, ALUOp, Branch, MemWrite, MemToReg, Jal, Jr} = {1'b1, 1'b1, 1'b0, 2'b11, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};
	endcase
end
endmodule

//-----------------------------------------------------------
module ALUDecoder(
input[1:0] ALUOp,
input [4:0] Shamt,
input [5:0] Funct,
output reg [3:0] ALUControl
);
always @ (*) begin
	case (ALUOp)
	  2'b00 : ALUControl = 4'b0010;
	  2'b01 : ALUControl = 4'b0110;
	  2'b10 : begin
	  	if      ( Funct == 6'b100000 ) ALUControl = 4'b0010;
	  	else if ( Funct == 6'b100010 ) ALUControl = 4'b0110;
	  	else if ( Funct == 6'b100100 ) ALUControl = 4'b0000;
	  	else if ( Funct == 6'b100101 ) ALUControl = 4'b0001;
	  	else if ( Funct == 6'b101010 ) ALUControl = 4'b0111;
	  	else if ( Funct == 6'b000100 ) ALUControl = 4'b1010;  // sllv
	  	else if ( Funct == 6'b000110 ) ALUControl = 4'b0101;  // srlv
	  	else if ( Funct == 6'b000111 ) ALUControl = 4'b1111;  // srav
	  end
	  2'b11 : begin
		if      (( Funct == 6'b010000 ) && ( Shamt == 5'b00000 )) ALUControl = 4'b1000;
	  else if (( Funct == 6'b010000 ) && ( Shamt == 5'b00100 )) ALUControl = 4'b1001;	  	
	  end
	endcase
end
endmodule
//-----------------------------------------------------------
module ControlUnit(
input [5:0] Opcode,
input [5:0] Funct,
input [4:0] Shamt,
output[3:0] ALUControl,
output RegWrite, RegDst, ALUSrc, Branch, MemWrite, MemToReg, Jal, Jr
);

wire [1:0] WIREALUOp;

MainDecoder myDecoder1(Opcode, RegWrite, RegDst, ALUSrc, WIREALUOp, Branch, MemWrite, MemToReg, Jal, Jr);
ALUDecoder  myDecoder2(WIREALUOp, Shamt, Funct, ALUControl);
endmodule