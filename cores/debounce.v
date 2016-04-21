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
// Engineer: 	Jennifer Holt
// 
// Create Date:    09:54:59 11/21/2015 
// Design Name: 
// Module Name:    debounce 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: Simple switch debouncer. Parameter NBITS determines the size of the timeout counter. (17 bits for 48MHz = 2.7ms)
//
// Dependencies: None
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module debounce #(
	parameter NBITS = 16,
	parameter INVERT = "True"
	)(
    input wire clk,
    input wire I,
    output reg O,
    output wire U,
    output wire D
    );

	//input registers
	reg [1:0] Iq;
	generate
		if (INVERT == "TRUE")
			always @(posedge clk) begin
				Iq[1] <= Iq[0];
				Iq[0] <= ~I;
			end
		else
			always @(posedge clk) begin
				Iq[1] <= Iq[0];
				Iq[0] <= I;
			end
	endgenerate
	
	//counter
	reg [NBITS-1:0] count;
	
	// logic
	wire Idle = (O == Iq[1]);
	wire count_max = &count;		// true when all bits of count are 1's

	always @(posedge clk)
		if(Idle)
			count <= 0;  // nothing's going on
		else	begin
			count <= count + 1'd1;  // something's going on, increment the counter
			if(count_max) O <= ~O;  // if the counter is maxed out, PB changed!
		end

	assign D = ~Idle & count_max & ~O;
	assign U = ~Idle & count_max &  O;
endmodule
