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
// Create Date:    10:19:18 07/14/2015 
// Design Name: 
// Module Name:    commFPGA_fx2wrapper 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module commFPGA_fx2wrapper(
		input  wire      	fx2Clk,	    	// externally buffered 48MHz clock from FX2

		// FX2 interface -----------------------------------------------------------------------------
		output wire[1:0] 	fx2Addr_out,  	// select FIFO: "10" for EP6OUT, "11" for EP8IN
		inout  wire[7:0] 	fx2Data_io,   	// 8-bit data to/from FX2

		// When EP6OUT selected:
		output wire      	fx2Read_out,  	// asserted (active-low) when reading from FX2
		output wire      	fx2OE_out,    	// asserted (active-low) to tell FX2 to drive bus
		input  wire      	fx2GotData_in,	// asserted (active-high) when FX2 has data for us

		// When EP8IN selected:
		output wire      	fx2Write_out, 	// asserted (active-low) when writing to FX2
		input  wire      	fx2GotRoom_in,	// asserted (active-high) when FX2 has room for more data from us
		output wire     	fx2PktEnd_out,	// asserted (active-low) when a host read needs to be committed early
		
		// FPGA interface-----------------------------------------------------------------------------
		output wire 	  	fx2Reset,
		output wire[6:0] 	chanAddr,  // the selected channel (0-127)

		// Host >> FPGA pipe:
		output wire[7:0]	h2fData,		// data lines used when the host writes to a channel
		output wire		h2fValid,	// '1' means "on the next clock rising edge, please accept the data on h2fData_out"
		input wire		h2fReady,
	
		// Host << FPGA pipe:
		input wire[7:0]	f2hData,		// data lines used when the host reads from a channel
		output wire		f2hReady,	// '1' means "on the next clock rising edge, put your next byte of data on f2hData_in"
		input wire		f2hValid
    );

	// Needed so that the comm_fpga_fx2 module can drive both fx2Read_out and fx2OE_out
		wire       fx2Read;
	// CommFPGA module
		assign fx2Read_out = fx2Read;
		assign fx2OE_out = fx2Read;
		assign fx2Addr_out[0] =  // So fx2Addr_out[1]='0' selects EP2OUT, fx2Addr_out[1]='1' selects EP6IN
			(fx2Reset == 1'b0) ? 1'b0 : 1'bZ;

	//commfpga module
	comm_fpga_fx2 FPGALink(
		.clk_in(fx2Clk),
		.reset_in(1'b0),
		.reset_out(fx2Reset),
		
		// FX2LP interface
		.fx2FifoSel_out(fx2Addr_out[1]),
		.fx2Data_io(fx2Data_io),
		.fx2Read_out(fx2Read),
		.fx2GotData_in(fx2GotData_in),
		.fx2Write_out(fx2Write_out),
		.fx2GotRoom_in(fx2GotRoom_in),
		.fx2PktEnd_out(fx2PktEnd_out),

		// DVR interface -> Connects to application module
		.chanAddr_out(chanAddr),
		.h2fData_out(h2fData),
		.h2fValid_out(h2fValid),
		.h2fReady_in(h2fReady),
		.f2hData_in(f2hData),
		.f2hValid_in(f2hValid),
		.f2hReady_out(f2hReady)
	);

endmodule
