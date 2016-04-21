`timescale 1ns / 1ps
`default_nettype none
// Copyright (c) 2016 Jennifer Holt
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
// OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:23:51 07/22/2013 
// Design Name: 
// Module Name:    configuration_memory 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//		Configuration register memory for FPGALink applications. register width, number of registers and base address are set by parameters.
//		Direct connection to commFPGA DVR pipes. registers are read/write from host side and read only from application side.
//		One chanAddr from commFPGA is used for each register, and an offset in chanAddr is set by BASEADDR, for example:
//		NREG = 8, NBYTES = 4, BASEADDR = 5 would create 8 32-bit registers starting at chanAddr 5.
//		
//		Implements an (NREG*NBYTES) byte wide memory as flip-flops. Read/Write access is provided on a byte wide bus, and continuous parallel read is provided on all bits.
//		Memory is arranged internally as an array of bytes with a byte memory pointer. The pointer is set to a register boundary at the start of every read/write.
//		The memory pointer increments on each byte read/written (note a transaction larger than NBYTES will start reading/writing into the next register)
//		Register values are output on PDout as:
//		{{REGn:m[7:0], REGn:m-1[7:0] ... REGn:0[7:0]}, {REGn-1:m[7:0], REGn-1:m-1[7:0] ... REGn-1:0[7:0]}, {REG0:m[7:0], REG0:m-1[7:0] ... REG0:0[7:0]}}
//		with n = (NREG-1 .. 0) and m = (NBYTES-1 .. 0).
//
//		Reads/Writes are byte sequential starting from (chanAddr-BASEADDR)*NBYTES when rd/wr is asserted.
//		Connects directly to the DVR pipes provided by commFPGA (usually through a MUX).
//
//	Instantiation Template
//	configuration_memory #(.NREG(4), .NBYTES(4), .BASEADDR(0)) ConfigMem (
//			.fx2Clk		(fx2Clk), 
//			.reset		(fx2Reset), 
//			.chanAddr		(mem_chanAddr),
//			.h2fData		(mem_h2fData), 
//			.h2fValid		(mem_h2fValid), 
//			.h2fReady		(mem_h2fReady),
//			.f2hData		(mem_f2hData), 
//			.f2hValid		(mem_f2hValid),
//			.f2hReady		(mem_f2hReady), 
//			.PDout		({memReg3, memReg2, memReg1, memReg0}) 
//	);
//	
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//	internal memory pointer is 32-bit. unused bits should be optimized out by the synthesis tools.
//
//////////////////////////////////////////////////////////////////////////////////
   
module configuration_memory #(parameter NREG = 1, parameter NBYTES = 4, parameter[6:0] BASEADDR = 0)(
	input wire fx2Clk,
	input wire reset,
	 
	input wire [6:0] chanAddr,
	input wire [7:0] h2fData,
	input wire h2fValid,
	output wire h2fReady,
	 
	output wire [7:0] f2hData,
	input wire f2hReady,
	output wire f2hValid, 
	 
	output reg [NREG*NBYTES*8-1:0] PDout
    );
	
	//parameter POINTERBITS = $clog2(NREG*NBYTES);
	parameter POINTERBITS = 32;
	 
	assign h2fReady = 1'b1;			//always ready for writes
	assign f2hValid = 1'b1;			//output data is always valid
	
	//memory pointer
	reg [POINTERBITS-1:0] pointer = 0;
	wire [POINTERBITS-1:0] pointer_next;
	
	//wire for parallel data input to memory registers
	wire [NREG*NBYTES*8-1:0] PDin;
	
	//registered wr, rd
	reg wr_q = 1'b0;
	reg rd_q = 1'b0;
	
	//edge signals
	wire pedg_wr;
	wire pedg_rd;
	
	//edge logic
	assign pedg_wr = h2fValid & !wr_q;
	assign pedg_rd = f2hReady & !rd_q;
	
	//address logic
	wire AddrValid;
	wire [6:0] Addr;
	assign Addr = chanAddr-BASEADDR;
	assign AddrValid = (Addr >= 0) & (Addr < NREG);
	
	//pointer logic
	assign pointer_next = (pedg_wr | pedg_rd) ? (Addr) * NBYTES :
								 ((h2fValid | f2hReady) && (pointer == (NREG*NBYTES)-1)) ? 0 : 
								 ((h2fValid | f2hReady) && (pointer != (NREG*NBYTES)-1)) ? pointer + 1'b1 :
								 pointer;
	
	//write logic
	genvar i;
	generate
		for (i=0; i<(NREG*NBYTES) ; i=i+1) begin : test
			assign PDin[i*8+7:i*8] = (h2fValid & pointer_next == i) ? h2fData : PDout[i*8+7:i*8];
		end
	endgenerate
	
	//read logic
	wire [NREG*NBYTES*8-1:0] PDout_shift;
	assign PDout_shift = PDout >> ({3'b000,pointer_next} << 3);
	
	assign f2hData = (AddrValid) ? PDout_shift[7:0] : 8'd0;
			
	//registers
	always @ (posedge fx2Clk) begin
		if (reset) begin
			PDout <= 0;
			pointer <= 0;
			wr_q <= 0;
			rd_q <= 0;
		end else begin
			wr_q <= h2fValid;
			rd_q <= f2hReady;
			pointer <= pointer_next;
			PDout <= PDin;
		end
	end


endmodule
