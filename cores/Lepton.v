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
// Create Date:    23:03:53 09/21/2015 
// Design Name: 
// Module Name:    Lepton
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: CommFPGA based interface core for FLIR Lepton cameras.
//
// chanAddr map:
// BASEADDR+0		Config register. controls resets and formatting. Camera formatting must be set with the CCI interface. See Note1 for bits.
// BASEADDR+1		Pixel stream FIFO. A read will return bytes from the video stream. Will sync with the start of a frame on each access.
// BASEADDR+2		Telemetry FIFO. every telemetry sync, the fifo is reset and filled. can read up to 480 bytes. (3 telemetry packets)
// BASEADDR+3..8	I2Cmaster connected to the Lepton CCI interface. see CommFPGA_I2Cmaster.v for details.
//
// Note1
// Configuration register bits:
// 0		Lepton startup enable.
// 1		Lepton packet interface enable.
// 2		Lepton framer enable.
// 3		Pixel format, 0 = raw14, 1=RGB888. This must match the setting in the camera.
// 5:4	Telemetry mode. 2'b00 = disabled, 2'b01 = footer, 2'b10 = header, 2'b11 = reserved
// 6	     AGC mode, 0=AGC disabled, 1=AGC enabled. This must match the setting in the camera
// 7      reserved
//
// Dependencies: 
//			Lepton_packet.v, Lepton_framer.v, Lepton_pixelCounter.v
//			CommFPGA_I2Cmaster.v, CommFPGA_configMem.v, CommFPGA_mux.v
//			an independent clock, 8-bit wide FIFO with first word fall-through. Xilinx core generator can be used to make one.
//			an independent clock, 16-bit input, 8-bit output fifo with first word fall-through.
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module Lepton#(
	parameter BASEADDR = 0,				//Base address for the commFPGA interface
	parameter VERSION = 0,				//selects which camera version to use 0 = Lepton2, 1 = Lepton3
	parameter CDIV = 1,					//serial clock scaling value fsclk = fsysclk/(2*CDIV).
	parameter DELAY = 22'd3700000,		//sets the video timeout period. tTIMEOUT = DELAY*tSCLK (must be >185ms)
	parameter[15:0] DELAY2 = 16'd15000		//number of sysclk to count during lepton startup. should correspond to >5000 LClk
	)(
	input wire sysclk,					//system clock (defaults set up for 48MHz to match commFPGAlink interface)
	input wire reset,					//active high system reset

	//CommFPGAlink Interface
	input wire fx2Clk,
	input wire [6:0] chanAddr,
	input wire [7:0] h2fData,
	input wire h2fValid,
	output wire h2fReady,
	output wire [7:0] f2hData,
	input wire f2hReady,
	output wire f2hValid,
	
	//debuging output
     output wire[7:0] configReg0,       //monitor lines for config register
     output wire[7:0] PacketDebug,		//debug lines from Lepton_packet
     output wire[7:0] FramerDebug,      //debug lines from Lepton_framer
	
	//video output
	output wire vsync,	     		//vertical sync pulse
	output wire hsync,  			//horizontal sync pulse
	output wire pvalid,				//strobe for each pixel
	output wire [23:0] pixel,		//pixel data {RR,GG,BB} or {10'hxxx, 14'hnnnnn}
    
	//telemetry output
	output wire tsync,				//strobe at beginning of telemetry data. total of 80 words
	output wire tvalid,				//strobe for each telemetry data word
	output wire [15:0] tdata,		//16-bit telemetry word.	
	
	//Lepton connections
	output wire L_nReset,			//Lepton active low reset
	output wire L_Mclk_en,			//Lepton Master Clock enable
	output wire L_nPwd,				//Lepton active low power down
	inout wire L_Scl,				//Lepton I2C clock
	inout wire L_Sda,				//Lepton I2C data
	input wire L_fsync,				//Lepton frame ready signal (GPIO3)
	output wire L_nCs,				//Lepton VOSPI active low chip select
	output wire L_Clk,				//Lepton VOSPI clock
	input wire L_Miso				//Lepton VOSPI data
    );

	//Address allocation logic
	//This logic allocates commFPGA chanAddr addresses to the mux channels
	wire[7:0] muxAddr;
	assign muxAddr	=	(chanAddr == BASEADDR)					?	8'd1			:		//config mem at BASEADDR+0
					(chanAddr == BASEADDR+1)					?	8'd2			:		//pixel stream at BASEADDR+1
					(chanAddr == BASEADDR+2)					?	8'd3			:		//telemetry mem at BASEADDR+2
					(chanAddr < BASEADDR+9)					?	8'd4			:		//I2Cmaster at BASEADDR+(3..8)
					 8'd0;
	
	//muxed commFPGA channels to other modules
	wire[6:0] config_chanAddr;
	wire[7:0] config_h2fData;
	wire config_h2fValid;
	wire config_h2fReady;
	wire[7:0] config_f2hData;
	wire config_f2hValid;
	wire config_f2hReady;
	
	wire[6:0] stream_chanAddr;
	wire[7:0] stream_h2fData;
	wire stream_h2fValid;
	wire stream_h2fReady;
	wire[7:0] stream_f2hData;
	wire stream_f2hValid;
	wire stream_f2hReady;
	
	wire[6:0] tm_chanAddr;
	wire[7:0] tm_h2fData;
	wire tm_h2fValid;
	wire tm_h2fReady;
	wire[7:0] tm_f2hData;
	wire tm_f2hValid;
	wire tm_f2hReady;
	
	wire[6:0] I2C_chanAddr;
	wire[7:0] I2C_h2fData;
	wire I2C_h2fValid;
	wire I2C_h2fReady;
	wire[7:0] I2C_f2hData;
	wire I2C_f2hValid;
	wire I2C_f2hReady;
	
	//mux switches commFPGA interface to multiple modules based on .mux input. .mux = 0 links commFPGA to a null sink(always accepts write, always reads 0)
	//No base address subtraction is done, so mux'd modules should have the proper base adress set.
	commFPGA_mux #(.NMUX(4)) FPGALink_mux (
		.mux			(muxAddr),
		
		//input from commFPGA module
		.chanAddr		(chanAddr),
		.h2fData		(h2fData),
		.h2fValid		(h2fValid),
		.h2fReady		(h2fReady),
		.f2hData		(f2hData),
		.f2hValid		(f2hValid),
		.f2hReady		(f2hReady),
		
		//muxed output to application
		.chanAddr_mux	({I2C_chanAddr, tm_chanAddr, stream_chanAddr, config_chanAddr}),
		.h2fData_mux	({I2C_h2fData, tm_h2fData, stream_h2fData, config_h2fData}),
		.h2fValid_mux	({I2C_h2fValid, tm_h2fValid, stream_h2fValid, config_h2fValid}),
		.h2fReady_mux	({I2C_h2fReady, tm_h2fReady, stream_h2fReady, config_h2fReady}),
		.f2hData_mux	({I2C_f2hData, tm_f2hData, stream_f2hData, config_f2hData}),
		.f2hValid_mux	({I2C_f2hValid, tm_f2hValid, stream_f2hValid, config_f2hValid}),
		.f2hReady_mux	({I2C_f2hReady, tm_f2hReady, stream_f2hReady, config_f2hReady})
	);
	
	//config memory for for control
	configuration_memory #(.NREG(1), .NBYTES(1), .BASEADDR(BASEADDR)) ConfigMem (
			.fx2Clk		(fx2Clk), 
			.reset		(reset), 
			.chanAddr		(config_chanAddr),
			.h2fData		(config_h2fData), 
			.h2fValid		(config_h2fValid), 
			.h2fReady		(config_h2fReady),
			.f2hData		(config_f2hData), 
			.f2hValid		(config_f2hValid),
			.f2hReady		(config_f2hReady), 
			.PDout		(configReg0) //these output lines could be used directly by other logic, but intermediate wires make the code easy to read.
	);
	
	//FIFO for pixel data stream
	//need to intercept pixel data at the byte level, between the packet interface and the framer
	//USB should be much faster than the pixel rate, so the FIFO can be minimally deep
	
	reg fread_sync;
	//this reg ensures that reads always start at the beginning of a frame
	always @(posedge sysclk) begin
		if(!stream_f2hReady) begin
			fread_sync <= 1'b0;
		end else begin
			if (vsync) fread_sync <= 1'b1;
			else fread_sync <= fread_sync;
		end
	end
	
	wire [7:0] d_byte;
	wire fifo_wr, fifo_rd, fifo_empty, valid;
	assign stream_f2hValid = !fifo_empty;					//data is valid if the fifo isn't empty
	assign fifo_rd = stream_f2hReady;						//clock data out of the fifo as long as we're reading
	//write to fifo if: read is requested and the byte is valid and the packet is normal and the frame has been synced
	assign fifo_wr = stream_f2hReady & valid & normal & fread_sync;
	assign stream_h2fReady = 1'b1;						//don't block writes to this chanAddr even though they are ignored.
	
	ICfifo8_fallthrough Pixel_FIFO (
		.rst(!stream_f2hReady),	 		// input rst
		.wr_clk(sysclk), 				// input wr_clk
		.rd_clk(fx2Clk), 				// input rd_clk
		.din(d_byte),					// input [7 : 0] din
		.wr_en(fifo_wr), 				// input wr_en
		.rd_en(fifo_rd),	 			// input rd_en
		.dout(stream_f2hData),			// output [7 : 0] dout
		.full(), 						// output full
		.empty(fifo_empty) 				// output empty
	);
	//fifo for telemetry
	wire tm_empty;
	assign tm_f2hValid = !tm_empty;
	ICfifo16x8_fallthrough Telemetry_FIFO (
		.rst(tsync | reset), // input rst
		.wr_clk(sysclk), // input wr_clk
		.rd_clk(fx2Clk), // input rd_clk
		.din(tdata), // input [15 : 0] din
		.wr_en(tvalid), // input wr_en
		.rd_en(tm_f2hReady), // input rd_en
		.dout(tm_f2hData), // output [7 : 0] dout
		.full(), // output full
		.empty(tm_empty) // output empty
	);
	assign tm_h2fReady = 1'b1;
		
	//I2C master for CCI
	CommFPGA_I2Cmaster #(.BASEADDR(BASEADDR+3), .PRER(16'h19)) CCI (
		.fx2Clk	(fx2Clk),
		.reset	(reset),
	
		.chanAddr	(I2C_chanAddr),
		.h2fData	(I2C_h2fData), 
		.h2fValid	(I2C_h2fValid), 
		.h2fReady	(I2C_h2fReady),
		.f2hData	(I2C_f2hData), 
		.f2hValid	(I2C_f2hValid),
		.f2hReady	(I2C_f2hReady),

		.SCL		(L_Scl),
		.SDA		(L_Sda)
	);
	
	//Lepton Startup sequence
	`ifndef SYNTHESIS
		parameter STARTCOUNTEND = 16'd8;
	`else
		parameter STARTCOUNTEND = DELAY2;		
	`endif
	
	reg [15:0] Start_Count;						//startup counter, need to wait at least 5000 LClk = 5000 * 1/25MHz = 200us
	assign L_nPwd = Start_Count > 1;				//Power Down de-asserted
	assign L_Mclk_en = Start_Count > 4;			//Master clock enabled
	assign L_nReset = Start_Count == STARTCOUNTEND;	//wait 5000 LClk then reset de-asserted
		
	always @(posedge sysclk) begin
		if (reset | !configReg0[0]) begin
			Start_Count <= 0;
		end else begin
			if (Start_Count != STARTCOUNTEND) Start_Count <= Start_Count + 1;
			else Start_Count <= Start_Count;
		end
	end
	
	//Lepton packet interface
	wire sync, stuck;
     wire discard, normal, special, n_locked;
	wire [15:0] id, crc;
     wire [1:0] t_packets;
     assign PacketDebug[3] = n_locked;
     assign PacketDebug[4] = normal;
     generate
          if (VERSION == 0) begin
               assign t_packets = (configReg0[5:4] ==0 ) ? 2'd0 : 2'd3;    //telemetry adds 3 packets to each frame for Lepton2
          end else begin
               assign t_packets = (configReg0[5:4] ==0 ) ? 2'd0 : 2'd1;    //telemetry adds 1 packet to each segment for Lepton3
          end
     endgenerate
     
	Lepton_packet #(
		.CDIV(CDIV),
		.DELAY(DELAY)
	)VOSPI(
		.sysclk	(sysclk), 
		.reset_in	(reset | !configReg0[1] | !L_nReset), 
		.fpixel	(configReg0[3]),
          .telemetry(t_packets),
          //debug outputs
          .reset    (PacketDebug[0]),                                     //internal reset signal
		.packet50	(PacketDebug[1]), 
		.sdone    (PacketDebug[2]),
          .seg		(PacketDebug[7:5]),
          //packet outputs
		.sync	(sync),     
		.id		(id), 
		.crc		(crc), 
		.valid	(valid), 
		.d_byte	(d_byte),
          .n_ready  (n_locked),
		.discard	(discard),
		.normal	(normal),
		.special	(special),
		.stuck	(stuck),
		.nCs		(L_nCs), 
		.sClk	(L_Clk), 
		.Miso	(L_Miso)
	);
	
	//Lepton video framer
	wire [23:0] pix;
     assign FramerDebug[0] = reset | n_locked | !configReg0[2];
	Lepton_framer #(.VERSION(VERSION)) Framer(
		.sysclk		(sysclk),
		.reset		(reset | n_locked | !configReg0[2]),		     //reset when the packet interface is not locked
	
	//format inputs
		.fpixel		(configReg0[3]),		//pixel format 0= raw14, 1= RGB888 (must match settings on camera and psize on Lepton_packet)
		.telemetry	(configReg0[5:4]),		//telemetry mode. 2'b00 = disabled, 2'b01 = footer, 2'b10 = header, 2'b11 = reserved
	
	//inputs from Lepton_packet
		.sync		(sync),
		.id			(id),
		.crc			(crc),
		.valid		(valid),
		.d_byte		(d_byte),
		.discard		(discard),
		.normal		(normal),
		.special		(special),
	
	//video output
		.vsync		(vsync),				//strobe at the beginning of each full frame
		.hsync		(hsync),				//strobe at the beginning of each line(not used)
		.pvalid		(pvalid),				//strobe for each pixel
		.pixel		(pix),				//pixel data {RR,GG,BB} or {10'hxxx, 14'hnnnnn}
	
	//telemetry output
		.tsync		(tsync),				//strobe at beginning of telemetry data
		.tvalid		(tvalid),				//strobe for each telemetry data word
		.tdata		(tdata),				//16-bit telemetry word.
		.blank		(FramerDebug[1])
	);
     
     //output pixel mapping
     assign pixel = (configReg0[3]) ?  pix :
                                      (configReg0[6]) ? {pix[7:0], pix[7:0], pix[7:0]} : {10'd0, pix[13:0]};
endmodule
