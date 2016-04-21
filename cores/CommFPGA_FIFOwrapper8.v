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
// Create Date:    11:45:30 07/15/2015 
// Design Name: 
// Module Name:    CommFPGA_FIFOwrapper8 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//	contains two 8-bit FIFOS and connection logic to hook it up to commFPGA systems.
// convention is the inFIFO is Host => FPGA and the outFIFO is FPGA => host
//
// Dependencies: common clock 8-bit FIFO with first-word fall-through and 8-bit data count.  
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module CommFPGA_FIFOwrapper8(
    input wire fx2Clk,
    input wire reset,
	 
//  input wire [6:0] chanAddr,
    input wire [7:0] h2fData,
    input wire h2fValid,
    output wire h2fReady,
    output wire [7:0] f2hData,
    output wire f2hValid,
    input wire f2hReady,
	 
	 input wire rd,
	 input wire wr,
	 input wire[7:0] Din,
	 output wire[7:0] Dout,
	 output wire[7:0] inCount,
	 output wire[7:0] outCount,
	 output wire full,
	 output wire empty 
    );
	 
	wire inFull;
	wire dummy1;
	wire dummy2;
	assign h2fReady = !inFull;
	
	CCfifo8_fallthrough inFIFO (
		.clk(fx2Clk), // input clk
		.rst(reset), // input rst
		.din(h2fData), // input [7 : 0] din
		.wr_en(h2fValid), // input wr_en
		.rd_en(rd), // input rd_en
		.dout(Dout), // output [7 : 0] dout
		.full(inFull), // output full
		.empty(empty), // output empty
		.data_count({inCount}) // output [7 : 0] data_count
	);

	wire outEmpty;
	assign f2hValid = !outEmpty;
	
	CCfifo8_fallthrough outFIFO (
		.clk(fx2Clk), // input clk
		.rst(reset), // input rst
		.din(Din), // input [7 : 0] din
		.wr_en(wr), // input wr_en
		.rd_en(f2hReady), // input rd_en
		.dout(f2hData), // output [7 : 0] dout
		.full(full), // output full
		.empty(outEmpty), // output empty
		.data_count({outCount}) // output [7 : 0] data_count
	);

endmodule
