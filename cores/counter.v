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
// Engineer:       Jennifer Holt
// 
// Create Date:    13:05:13 02/06/2016 
// Design Name: 
// Module Name:    strobe_counter 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: Counter with enable strobe.
//
// Dependencies: None
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module counter#(
     parameter NBITS = 22
     )(
     input wire sysclk,
     input wire reset,
     input wire up,
     input wire dn,
     input wire[NBITS-1:0] max,
     output reg[NBITS-1:0] count
     );

     wire[NBITS-1:0] count_next;
     wire[NBITS-1:0] count_up, count_down;
     wire [NBITS:0] count_plus1;
	
     assign count_plus1 = count + 1;
     assign count_up = (count == max) ? 0 : count_plus1[NBITS-1:0];
     assign count_down = (count == 0) ? max : count - 1;
     assign count_next = (up) ? count_up   : 
                         (dn) ? count_down :
                                count;
     
     always @(posedge sysclk) begin
          if (reset) begin
               count <= 0;
          end else begin
               count <= count_next;
          end
     end
endmodule
