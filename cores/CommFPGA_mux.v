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
// Create Date:    14:54:53 07/14/2015 
// Design Name: 
// Module Name:    commFPGA_mux 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description:
//			Creates a mux for commFPGA DVR interface channels. the .mux input determines which output is connected to the commFPGA module.
//			.mux input of 0 disconnects the commFPGA module from all output modules. While .mux = 0, any reads will return 0, any writes will not block.
//
// Dependencies: 
//
// Instantiation template
//	commFPGA_mux #(.NMUX(3)) FPGALink_mux (
//		.mux				(muxAddr),
		
//		//input from commFPGA module
//		.chanAddr		(chanAddr),
//		.h2fData		(h2fData),
//		.h2fValid		(h2fValid),
//		.h2fReady		(h2fReady),
//		.f2hData		(f2hData),
//		.f2hValid		(f2hValid),
//		.f2hReady		(f2hReady),
		
//		//muxed output to application
//		.chanAddr_mux	({module3_chanAddr, module2_chanAddr, module1_chanAddr}),
//		.h2fData_mux	({module3_h2fData, module2_h2fData, module1_h2fData}),
//		.h2fValid_mux	({module3_h2fValid, module2_h2fValid, module1_h2fValid}),
//		.h2fReady_mux	({module3_h2fReady, module2_h2fReady, module1_h2fReady}),
//		.f2hData_mux	({module3_f2hData, module2_f2hData, module1_f2hData}),
//		.f2hValid_mux	({module3_f2hValid, module2_f2hValid, module1_f2hValid}),
//		.f2hReady_mux	({module3_f2hReady, module2_f2hReady, module1_f2hReady})
//	);
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module commFPGA_mux #(parameter NMUX = 3)(
	input wire[7:0] mux,
	
	//input from commFPGA module
	input wire[6:0] chanAddr,
	input wire[7:0] h2fData,
	input wire h2fValid,
	output wire	h2fReady,
	output wire[7:0] f2hData,
	output wire f2hValid,
	input wire f2hReady,
	
	//muxed output to application
	output wire[NMUX*7-1:0]	chanAddr_mux,
	output wire[NMUX*8-1:0]	h2fData_mux,
	output wire[NMUX-1:0] 	h2fValid_mux,
	input wire[NMUX-1:0]	h2fReady_mux,
	input wire[NMUX*8-1:0]	f2hData_mux,
	input wire[NMUX-1:0]	f2hValid_mux,
	output wire[NMUX-1:0]	f2hReady_mux
	);
	
	generate
		genvar i;
		for (i=0; i < NMUX ; i=i+1) begin : test
			//assignments from commFPGA to modules
			assign chanAddr_mux[i*7+6:i*7]	=	(mux == i+1)	?	chanAddr :	6'b000000;
			assign h2fData_mux[i*8+7:i*8]		=	(mux == i+1)	?	h2fData	:	8'b00000000;
			assign h2fValid_mux[i]				=	(mux == i+1)	?	h2fValid	:	1'b0;
			assign f2hReady_mux[i]				=	(mux == i+1)	?	f2hReady	:	1'b0;				//when a mux channel is not connected, make sure that the ready line is 0
		end
	endgenerate
	
	//assignments from modules to commFPGA
	wire [NMUX-1:0] h2fReady_shift;
	assign h2fReady_shift = h2fReady_mux >> (mux - 1);
	wire[NMUX*8-1:0] f2hData_shift;
	assign f2hData_shift = f2hData_mux >> ({3'b000,(mux - 1)} << 3);
	wire [NMUX-1:0] f2hValid_shift;
	assign f2hValid_shift = f2hValid_mux >> (mux - 1);
	
	assign h2fReady	=	(mux == 8'd0)	?	1'b1	:	h2fReady_shift[0];		//for mux==0 don't block writes
	assign f2hData		=	(mux == 8'd0)	?	8'd0	:	f2hData_shift[7:0];		//for mux==0 read 0
	assign f2hValid	=	(mux == 8'd0)	?	1'b1	:	f2hValid_shift[0];		//for mux==0 data is always valid

endmodule
