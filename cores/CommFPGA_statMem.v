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
// Module Name:    status_memory 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//		Status register(s)  for FPGALink applications. register width, number of registers and base address are set by parameters.
//		Direct connection to commFPGA DVR pipes. registers values are taken from application side and read only from host side.
//		reads start at (chanAddr-BASEADDR)*NBYTES and are sequential, writes do not block commFPGA, but are ignored.
//		status input values are input on PDin as:
//		{{REGn:m[7:0], REGn:m-1[7:0] ... REGn:0[7:0]}, {REGn-1:m[7:0], REGn-1:m-1[7:0] ... REGn-1:0[7:0]}, {REG0:m[7:0], REG0:m-1[7:0] ... REG0:0[7:0]}}
//		with n = (NREG-1 .. 0) and m = (NBYTES-1 .. 0)
//		
//		Easily connects to the DVR pipes provided by commFPGA:
//
//	Instantiation Template
//	status_memory #(.NREG(4), .NBYTES(4), .BASEADDR(0)) StatMem (
//			.fx2Clk		(fx2Clk), 
//			.reset		(fx2Reset), 
//			.chanAddr	(stat_chanAddr),
//			.h2fData		(stat_h2fData), 
//			.h2fValid	(stat_h2fValid), 
//			.h2fReady	(stat_h2fReady),
//			.f2hData		(stat_f2hData), 
//			.f2hValid	(stat_f2hValid),
//			.f2hReady	(stat_f2hReady), 
//			.PDin			({statReg3, statReg2, statReg1, statReg0}), 
//	);
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module status_memory #(parameter NREG = 4, parameter NBYTES = 4, parameter[6:0] BASEADDR = 0)(
	input wire fx2Clk,
	input wire reset,
	 
	input wire [6:0] chanAddr,
//	input wire [7:0] h2fData,
//	input wire h2fValid,
	output wire h2fReady,
	
	output wire [7:0] f2hData,
	output wire f2hValid,
	input wire f2hReady,
	 
	input wire [NREG*NBYTES*8-1:0] PDin
    );

	assign h2fReady = 1'b1;			//although writes are ignored, don't block them.
	
	//memory pointer
	reg [31:0] pointer = 0;
	wire [31:0] pointer_next;
	
	//registered rd
	reg rd_q = 1'b0;
	
	//edge signals
	wire pedg_rd;
	
	//edge logic
	assign pedg_rd = f2hReady & !rd_q;
	
	//address logic
	wire AddrValid;
	wire [6:0] Addr;
	assign Addr = chanAddr-BASEADDR;
	assign AddrValid = (Addr >= 0) & (Addr < NREG);
	
	//pointer logic
	assign pointer_next = (pedg_rd) ? Addr * NBYTES :
								 (f2hReady && (pointer == (NREG*NBYTES)-1)) ? 0 : 
								 (f2hReady && (pointer != (NREG*NBYTES)-1)) ? pointer + 1 :
								  pointer;

	//read logic
	wire [NREG*NBYTES*8-1:0] PDin_shift;
	assign PDin_shift = PDin >> ({3'b000,pointer_next} << 3);
	
	assign f2hData = (AddrValid) ? PDin_shift[7:0] : 8'd0;
	//assign f2hValid = AddValid;	//This one will block on reads outside range
	assign f2hValid = 1'b1;
	
	//registers
	always @ (posedge fx2Clk) begin
		if (reset) begin
			pointer <= 0;
			rd_q <= 0;
		end else begin
			rd_q <= f2hReady;
			pointer <= pointer_next;
		end
	end

endmodule
