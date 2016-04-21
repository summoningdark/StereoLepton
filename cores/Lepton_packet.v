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
// Create Date:    08:38:04 11/07/2015 
// Design Name: 
// Module Name:    Lepton_packet 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: VOSPI packet interface for FLIR Lepton cameras. Syncs to the VOSPI channel and outputs packet data and timing signals.
//              Parameter for setting the serial clock rate(Lepton Max is 20MHz). Inputs for selecting 164 vs 244 bytes/packet.
//              Automatically detects loss of sync by illegal/out-of-order packet headers, and self resets.
//              Requires number of telemetry packets as input to correctly identify out of order packets.
//              if telemetry is enabled, Lepton2 adds 3 telemetry packets, Lepton3 adds one telemetry packet.
//
// Output timing:
// sysclk  __--__--__--| |__--__--| |__--__--| ~ |__--__--| |__--__--| |__--__--__--__--| |__--__--|
// sync    ____________| |__----__| |________| ~ |________| |________| |__----__________| |________|
// id      xxxxx{ID0===| |========| |========| ~ |========| |=={ID1==| |================| |========|
// crc     xxxxxxxxxxxx| |x{CRC0==| |========| ~ |========| |========| |={CRC1==========| |========|
// valid   ____________| |________| |__----__| ~ |__----__| |________| |________________| |__----__|
// d_byte  xxxxxxxxxxxx| |xxxxxxxx| |x{Byte0=| ~ |={ByteN=| |========| |={xxxxxxxxxxxxxx| |x{Byte0=|
// n_ready ------------| |--______| |________| ~ |________| |________| |________________| |________|
// discard xxxxx{d=====| |========| |========| ~ |========| |=={d====| |================| |========|
// normal  xxxxx{n=====| |========| |========| ~ |========| |=={n====| |================| |========|
// special xxxxx{======| |========| |========| ~ |========| |=={s====| |================| |========|
// if fpixel is high N = 243, if fpixel is low N=163.
//
// use sync as a strobe to read id and crc into other cores.
// use valid to read data bytes into other cores.
// n_ready is used as an active high reset to downstream cores.
// discard/normal/special indicate what type of packet is currently being read.
//
// Dependencies: clock_divider.v
//               counter.v
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments:
//                     id/discard/normal/special are valid on sync strobe, but change 16 sclk eariler(as soon as the ID is read)
// 
//////////////////////////////////////////////////////////////////////////////////
module Lepton_packet#(
	parameter[3:0] CDIV = 1,				//sets the serial clock. fSCLK = fSYSCLK/(2*(CDIV+1)). serial clock should be >8.5MHz and <20MHz
	parameter[21:0] DELAY = 22'd3700000,	//sets the video timeout period. tTIMEOUT = DELAY*tSCLK (must be >185ms)
	parameter NROW = 60,				//number of packets in a segment(without telemetry)
	parameter NCOL = 80					//number of pixels in a packet.
	)(
	input wire sysclk,					//system clock
	input wire reset_in,
	
	//control inputs
	input wire fpixel,					//pixel format input, high = read NCOL*3+4 byte packets, low = read NCOL*2+4 byte packets
	input wire [1:0] telemetry,			//number of extra packets due to telemetry
	
	//debug outputs
     output wire reset,                      //monitor for the internal reset signal
	output wire packet50,				//toggles each packet period (useful for external logic analyser)
     output wire sdone,                      //monitor for startup_done
	output wire [2:0] seg,				//segment number extracted from a special packet
	//data outputs
	output reg sync,					//strobe at the beginning of the packet (ID and CRC are valid on this edge)
	output reg [15:0] id,				//id field of the packet
	output reg [15:0] crc,				//crc value of the packet
	output reg valid,					//data strobe for d_byte, bytes are valide on this edge.
	output reg [7:0] d_byte,				//packet data bytes
	//status outputs
	output reg n_ready,					//goes low after 1 packet is read without errors, use as reset for next core in chain
	output reg discard,					//flags a discard packet
	output reg normal,					//flags a normal packet
	output reg special,					//flags a special packet
	output reg stuck,					//indicates that the VOSPI interface appears stuck. (MISO stays the same level for an entire packet)
	
	//lepton connections
	output reg nCs,					//Lepton VOSPI active low chip select
	output wire sClk,					//Lepton VOSPI clock
	input wire Miso					//Lepton VOSPI data
	);
     
     reg [3:0] reset_sfr;                    //shift register to generate internal reset pulse
	wire reset_int;                         //internal reset request.	
     assign reset = reset_in | (|reset_sfr);
               
     //serial clock
     wire sclk_pedg, sclk_nedg, sclk_int;
	clock_divider#(
         .NBITS(4), 
         .POL(1)
     ) SPICLK (
         .sysclk(sysclk),
	    .reset(reset),
         .div(CDIV),
	    .p_edge(sclk_pedg),
	    .n_edge(sclk_nedg),
	    .clk(sclk_int)
	);
     
	//counter. counts on serial clock negative edges. used to time the startup delay and count bits in the packet
	//maximum count needed is 185ms with a 20MHz SCLK => 3.7 million counts => 2^22 bits.
     parameter COUNTERBITS = 22;
     wire [COUNTERBITS-1:0] counter_max;
     wire[COUNTERBITS-1:0] counter;						
	counter#(
          .NBITS(COUNTERBITS)
     ) BitCounter (
          .sysclk(sysclk),
          .reset(reset),
          .up(sclk_nedg),               //count on negative edge of sClk so that count is stable for sampling edge
          .dn(1'b0),
          .max(counter_max),
          .count(counter)
     );
     	
	//startup sequence
	//per the lepton datasheet, nCS should be de-asserted for at least 185ms before attempting to sync the VOSPI data stream
	//after startup the counter is used to count bits in the packet
	wire[10:0] bitcount_max;
     assign bitcount_max =	(fpixel) ? (NCOL*3+4)*8-1 : (NCOL*2+4)*8-1;
	reg startup_done;								
	wire startup_done_next;
     assign sdone = startup_done;
	assign counter_max = (startup_done) ? bitcount_max : DELAY;
	assign startup_done_next = (startup_done) ? 1'b1 : (counter == DELAY & sclk_nedg);
	
     //serial interface
     wire nCs_next;
     assign nCs_next = (!nCs) ? 1'b0 : !(counter == DELAY & sclk_pedg);
     assign sClk = (startup_done) ? sclk_int : 1'b1;
     
	//input shift register
	reg [15:0] shift;
	wire [15:0] shift_next;
	assign shift_next = (sclk_pedg) ? {shift[14:0], Miso} : shift;
	
	//data capture
     wire id_strobe, crc_strobe, byte_strobe;
     wire valid_next, sync_next;
     wire [15:0] id_next, crc_next;
     wire[7:0] d_byte_next;
     wire data_region;
     
     assign id_strobe = counter == 15 & sclk_nedg & startup_done;               //strobe to capture id
     assign crc_strobe = counter == 31 & sclk_nedg & startup_done;              //strobe to capture crc
     assign data_region = |counter[10:5] & startup_done;                        //data region is anything greater than 31
     assign byte_strobe = (counter[2:0] == 3'd7) & sclk_nedg & data_region;     //strobe to capture data bytes
     assign id_next = (id_strobe) ? shift : id;                                 //capture id
     assign crc_next = (crc_strobe) ? shift : crc;                              //capture crc
     assign d_byte_next = (byte_strobe) ? shift[7:0] : d_byte;                  //capture byte data
     assign valid_next = byte_strobe;                                           //register byte strobe  
     assign sync_next = crc_strobe;                                             //register crc strobe and use for packet strobe
		
	//status flags
     reg bad_id, wrong_id;    //bad id flags an illegal ID field, wrong_id flags an ID which is out of order, (packet # != last # + 1)
     wire [12:0] next_pkt;    //ID+1, or 0 if id[11:0] == NROW-1 + telemetry.
	wire discard_next, normal_next, special_next, bad_id_next, wrong_id_next, n_ready_next;
	assign discard_next = (id_strobe) ? shift[11:8] == 4'hf : discard;
	assign normal_next =  (id_strobe) ? shift[11:0] < 63 : normal;
	assign special_next = (id_strobe) ? !shift[15] & shift[14:12] < 5 & shift[11:0] == 12'd20 : special;
     assign bad_id_next =  (id_strobe) ? !(discard_next | normal_next | special_next) : bad_id;
     assign next_pkt = ((id[11:0] == (NROW-1) + telemetry) | discard) ? 12'd0 : id[11:0] + 1;
     assign wrong_id_next =(id_strobe) ? !(shift[11:0] == next_pkt[11:0]) & normal_next : wrong_id;
     assign n_ready_next = (crc_strobe) ? 1'b0 : n_ready;
	
	//stuck serial input detection
//	reg first_bit;
//	reg stuck_test;
//	wire first_bit_next, stuck_next, stuck_test_next;
//	assign first_bit_next =	(bitcount == 0 & sclk_pedg) 	? Miso : first_bit;
//	assign stuck_test_next = (bitcount == 0 & sclk_nedg) 	? 1'b1 : 
//						(sclk_nedg)				? (first_bit == shift[0]) & stuck_test :
//						 stuck_test;
//	assign stuck_next = (bitcount == bitcount_max & sclk_nedg) ? stuck_test : stuck;
	
	//internal reset logic
     assign reset_int = (reset_in) ? 1'b0 : sync & (bad_id | wrong_id);	
	
     always @(posedge sysclk) begin
          reset_sfr <= {reset_sfr[2:0], reset_int};
     end
	
     //debug outputs
     reg toggle;
	wire toggle_next;
	assign toggle_next = (counter == counter_max & sclk_nedg) ? !toggle : toggle;
	assign packet50 = toggle;
     assign seg = (special) ? id[14:12] : 0;
     
	// infer registers
	always @(posedge sysclk) begin
		if (reset) begin
			startup_done <= 1'b0;
               sync <= 1'b0;
               valid <= 1'b0;
               nCs <= 1'b1;
			shift <= 0;
			id <= 0;
			crc <= 0;
			d_byte <= 0;
//			stuck <= 0;
//			stuck_test <= 0;
//			first_bit <= 0;
//			first_packet <= 0;
			toggle <= 0;
			discard <= 1'b0;
			normal <= 1'b0;
			special <= 1'b0;
               bad_id <= 1'b0;
               wrong_id <= 1'b0;
			n_ready <= 1'b1;
		end else begin
               nCs <= nCs_next;
			startup_done <= startup_done_next;
               sync <= sync_next;
               valid <= valid_next;
			shift <= shift_next;
			id <= id_next;
			crc <= crc_next;
			d_byte <= d_byte_next;
//			stuck <= stuck_next;
//			first_bit <= first_bit_next;
//			first_packet <= first_packet_next;
//			stuck_test <= stuck_test_next;
			toggle <= toggle_next;
			discard <= discard_next;
			normal <= normal_next;
			special <= special_next;
			bad_id <= bad_id_next;
               wrong_id <= wrong_id_next;
               n_ready <= n_ready_next;     
          end
	end
	
endmodule
