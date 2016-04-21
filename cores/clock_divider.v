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
// Create Date:    12:21:13 02/06/2016 
// Design Name: 
// Module Name:    clock_divider 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description:  Generates a slower clock from a fast system clock. includes output strobes for positive and negative edges.
//               The output frequency is calculated by fsysclk/(2*(DIV+1)). the output clock always has a 50% duty cycle.
// Dependencies: None
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module clock_divider#(
     parameter NBITS = 4,               //number of bits in the counter
     parameter [0:0] POL = 1'b0         //clock reset polarity
     )(
     input wire sysclk,
     input wire reset,
     input wire[NBITS-1:0] div,         //division factor. fclk = fsysclk/(2*(div+1))
     output wire p_edge,
     output wire n_edge,
     output reg clk
     );
    
     reg [NBITS-1:0] counter;
     wire [NBITS-1:0] counter_next;
     wire [NBITS:0] counter_plus1;
     assign counter_plus1 = counter + 1;
    
     wire edg;
     wire clk_next;
     assign edg = (counter == div);
     assign p_edge = (edg & !clk);
     assign n_edge = (edg & clk);
     assign counter_next = (edg) ? 0 : counter_plus1[NBITS-1:0];  //count for clock half-period
     assign clk_next = (edg) ? !clk : clk;

     always @(posedge sysclk) begin
          if (reset) begin
               counter <= 0;
               clk <= POL;
          end else begin
               counter <= counter_next;
               clk <= clk_next;
          end
     end
    

endmodule
