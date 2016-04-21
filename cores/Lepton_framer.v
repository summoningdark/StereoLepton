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
// Engineer: Jennifer Holt
// 
// Create Date:    13:41:53 11/07/2015 
// Design Name: 
// Module Name:    Lepton_framer 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: Takes the packet data stream from Lepton_packet and formats the data into a video stream.
//			 Has output stream for telemetry data if enabled. 
//              Inputs for selecting telemetry mode and pixel format. Parameter for Lepton Version.
//
// Output video timing: note no hsync pulse before row 0. hsync pulse preceds all rows 1 and higher.
// sysclk __--__--__--| |__--__--| ~ |__--__--| |__--__--| |__--__--| ~ |__--__--| ~~ |__--__--| |__--__--
// vsync  ______----__| |________| ~ |________| |________| |________| ~ |________| ~~ |________| |__----__
// hsync  ____________| |________| ~ |________| |__----__| |________| ~ |________| ~~ |________| |________
// pvalid ____________| |__----__| ~ |__----__| |________| |__----__| ~ |__----__| ~~ |__----__| |________
// pixel  ======xxxxxx| |x{p_0,0=| ~ |={p_c,0=| |========| |={p_0,1=| ~ |={p_c,1=| ~~ |={p_c,r=| |==xxxxxx
//
//
// Output tlemetry timing:
// sysclk __--__--__--| |__--__--| |__--__--| ~ |__--__--__| |__--__--
// tsync  ______----__| |________| |________| ~ |__________| |__----__
// tvalid ____________| |__----__| |__----__| ~ |__----____| |________
// tdata  ======xxxxxx| |x{word0=| |={word1=| ~ |={word239=| |==xxxxxx
//
// Dependencies: None
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module Lepton_framer#(
	parameter VERSION = 0,			//set VERSION=0 for Lepton2, VERSION != 0 for Lepton3
	parameter NPACKET = 60			//number of packets in a segment, without telemetry packets (for debugging)
	)(
	input wire sysclk,
	input wire reset,
	
	//format inputs
	input wire fpixel,				//pixel format 0= raw14, 1= RGB888 (must match settings on camera and fpixel on Lepton_packet)
	input wire [1:0] telemetry,		//telemetry mode. 2'b00 = disabled, 2'b01 = footer, 2'b10 = header, 2'b11 = reserved
	
	//inputs from Lepton_packet
	input wire sync,
	input wire [15:0] id,
	input wire [15:0] crc,
	input wire valid,
	input wire [7:0] d_byte,
	input wire discard,
	input wire normal,
	input wire special,
	
	//video output
	output wire vsync,				//strobe at the beginning of each full frame
	output wire hsync,				//strobe at the beginning of each line
	output wire pvalid,				//strobe for each pixel
	output wire [23:0] pixel,		//pixel data {RR,GG,BB} or {10'hxxx, 14'hnnnnn}
	
	//telemetry output
	output wire tsync,				//strobe at beginning of telemetry data
	output wire tvalid,				//strobe for each telemetry data word
	output wire [15:0] tdata,		//16-bit telemetry word.
	//debugging output
	output wire blank				//debug output
    );

	reg [23:0] byte_buffer;			//buffer to collect bytes into words
	wire [23:0] byte_buffer_next;
	reg valid_q, sync_q;			//delayed timing signals
	reg strobe2;					//toggle for strobe every 2 bytes
	reg [2:0] strobe3;				//counter for strobe every 3 bytes
	
	wire p3valid;					//single cycle strobe for a three byte word
	wire p2valid;					//single cycle strobe for a two byte word
	wire Vdata, Tdata;				//enables for video data or telemetry data
	
	//byte buffer logic
	assign byte_buffer_next = (valid) ? {byte_buffer[15:0], d_byte} : byte_buffer;
	always @(posedge sysclk) byte_buffer <= byte_buffer_next;
	
	//pixel mapping
	assign pixel = (fpixel) ? byte_buffer : {8'h00, byte_buffer[15:0]};
	
	//telemetry mapping
	assign tdata = byte_buffer[15:0];
	
	//data strobes
	assign p3valid = valid_q & strobe3[0];
	assign p2valid = valid_q & strobe2;
	
	always @(posedge sysclk) begin
		if (reset) begin
			valid_q <= 1'b0;
			sync_q <= 1'b0;
			strobe2 <= 1'b1;
			strobe3 <= 3'b001;
		end else begin
			valid_q <= valid;
			sync_q <= sync;
			if (sync) begin
				strobe2 <= 1'b1;
				strobe3 <= 3'b001;
			end else if (valid) begin
				strobe2 <= !strobe2;
				strobe3 <= {strobe3[1:0], strobe3[2]};
			end else begin
				strobe2 <= strobe2;
				strobe3 <= strobe3;
			end
		end
	end

	//this bit does the specifics for Lepton 2/3
	generate
		if (VERSION == 0) begin
			assign blank = 0;
			assign Tdata = !Vdata & !discard;
			assign pvalid = (fpixel) ? p3valid & Vdata : p2valid & Vdata;
			assign tvalid = Tdata & p2valid;
			assign Vdata = (telemetry == 2'b01)	?	!discard & id[11:0] < NPACKET		:	//footer
						(telemetry == 2'b10)	?	!discard & id[11:0] > 12'd2		:	//header
						 !discard;
			
			assign vsync = (telemetry == 2'b10)	?	sync_q & Vdata & id[11:0] == 12'd3	:	
						 sync_q & Vdata & id[11:0] == 12'd0;
			assign hsync = (telemetry == 2'b10)	?	sync_q & Vdata & id[11:0] > 12'd3	:
						 sync_q & Vdata & id[11:0] > 0;
			assign tsync = (telemetry == 2'b01)	?	sync_q & Tdata & id[11:0] == NPACKET	:
						 sync_q & Tdata & id[11:0] == 12'd0;
		end else begin
			reg LastSeg;						//flag for Segment 1 at packet 20.
			wire LastSeg_next;
			reg [11:0] SegSync;					//delay line to shift LastSeg pulse to start of valid frame
			wire [11:0] SegSync_next;
			reg blank_q;						//register to blank output during invalid frames
			wire blank_q_next;
			reg hsToggle;						//toggle bit to divide hsync by 2
			wire hsToggle_next, hsync1;
			
			always @(posedge sysclk) begin
				LastSeg <= LastSeg_next;
				SegSync <= SegSync_next;
				hsToggle <= hsToggle_next;
				blank_q <= blank_q_next;
			end
			
			//decode segment number. unfortunately segment number is reported on the 21st packet (id==20) so it is a bit late.
			//feed into a delay line to sync it up.
			assign LastSeg_next =	(reset) 								? 1'b0	:
								(sync_q & special & id[14:12] == 3'b001)	? 1'b1	:
								(sync_q & special & id[14:12] != 3'b000)	? 1'b0	:
								 LastSeg;
			
			//delay line delay by 12 to get to the next valid frame
			assign SegSync_next = 	(reset) ? 12'h000	:
								(sync & !discard & id[11:0] == 12'd0)	?	{SegSync[10:0], LastSeg}	:
								 SegSync;
								 
			//need a blanking signal which only allows valid frames through
			assign blank_q_next = (reset)			?	1'b1	:
							  (SegSync[11])	?	1'b0	:
							  (SegSync[3])		?	1'b1	:
							   blank_q;
			assign blank = (blank_q | SegSync[3]) & !SegSync[11];	//need to tweak the blanking period ends.
		
			assign Tdata = !Vdata & !discard & !blank;
			assign pvalid = (fpixel) ? p3valid & !blank & Vdata : p2valid & !blank & Vdata;
			assign tvalid = Tdata & p2valid;
			
			assign Vdata = (telemetry == 2'b01)	?	!discard & (id[11:0] < NPACKET-3 | !SegSync[2])	:	//footer
						(telemetry == 2'b10)	?	!discard & (id[11:0] > 12'd3 | !SegSync[11])		:	//header
						 !discard;

			assign vsync = (telemetry == 2'b10)	?	sync_q & !blank & id[11:0] == 12'd4 & SegSync[11]	:	
						 sync_q & !blank & id[11:0] == 12'd0 & SegSync[11];

			//Lepton3 hsync is more complicated, since there are two packets per video line
			assign hsToggle_next =	(reset)	?	1'b0		:
								(blank)	?	1'b0		:
                                        (vsync)   ?    1'b0      :
								(hsync1)	?	!hsToggle	:
								 hsToggle;
								 
	
			assign hsync1 = (telemetry == 2'b10)	?	sync_q & Vdata & !blank & (id[11:0] > 12'd4	| !SegSync[11]):
						 sync_q & !blank & Vdata;
			assign hsync = hsync1 & hsToggle;
		
			assign tsync = (telemetry == 2'b01)	?	sync_q & Tdata & id[11:0] == NPACKET-3 & SegSync[2]	:
						 sync_q & Tdata & id[11:0] == 12'd0 & SegSync[11];
		end
	endgenerate

endmodule
