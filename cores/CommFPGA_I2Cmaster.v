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
// Create Date:    13:39:01 07/17/2015 
// Design Name: 
// Module Name:    CommFPGA_I2Cmaster 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: FIFO based I2C master with commFPGA interface. Uses OpenCores i2c core (http://opencores.org/project,i2c)
// i2c sclk frequency set by parameter PRER. PRER = (fclk)/(5*fsclk) - 1. For 48MHz clk, 100kHz => PRER = 16'd95, 400kHz => PRER = 16'd19.
//
//
// To use 10-bit slave addresses set bit 10 in the slave address register. for 7-bit clear bit 10.
//
// i2c write transactions are accomplished by setting the slave address, setting the bytes to read to 0, and then writing bytes to the FIFO.
// writes to the i2c bus will continue until the FIFO is empty. It is assumed that the FPGAlink interface is much faster than
// the i2c bus, so the FIFO should never experience an underrun. Once all bytes have been added to the FIFO, poll the status register to
// detect the end of the transmission. If the core loses arbitration, the transfer is aborted and the appropriate bit is set in the status register.
//
// i2c read transactions are accomplished by setting the slave address and the number of bytes to read, 
// then writing any necessary command bytes to the FIFO(ie slave memory address etc). after all the bytes in the FIFO are sent,
// a repeated start condition is generated and the requested bytes are read into the FIFO. If the transaction is long it may be necessary
// read from the FIFO while the transfer is in progress to prevent overrun. Poll the status register or count bytes to detect the end.
// Like a write transaction, if arbitration is lost, the transfer is aborted and the approipriate bit is set in the status register.
//	
//	chanAddr			Description
// BASEADDR + 0		FIFO. bytes written are placed in the outgoing FIFO, bytes read are taken from the incoming FIFO. 
// BASEADDR + 1		number of bytes in the write FIFO 8-bit (read only)
// BASEADDR + 2		number of bytes in the read FIFO 8-bit (read only)
// BASEADDR + 3		Status 8-bit, see Note1
// BASEADDR + 4		Slave address 16-bit. {5'bXXXXX, 1'b[ten bit?], 10'b[slave address]} see Note2
// BASEADDR + 5		Bytes to Read 16-bit.	see Note2
//
// Note1:
// Status bits
// bit 0	ready, (1) core is idle and a transmission can be initiated, (0) core is busy, no new transmissions, however fifos are still active.
// bit 1 nak, read after transmission is complete if 1 transmission failed from slave not ack'ing
// bit 2 al, read after transmission is complete, if 1 transmission failed from loss of arbitration.
// bits 3..7, reserved
//
// Note 2:
// the config registers at BASEADDR + 4,5 will only accept values if the i2c Core is idle. this can be used as an alternate means of detecting
// bit 0 in the status flag, by monitoring the h2fReady signal of BASEADDR + 4,5.
//
//
//	Instantiation Template
//	CommFPGA_I2Cmaster #(.BASEADDR(0)) I2Cmaster (
//		.fx2Clk        (fx2Clk),
//		.reset         (Reset),
	
//		.chanAddr	     (i2c_chanAddr),
//		.h2fData	     (i2c_Din), 
//		.h2fValid	     (i2c_h2fValid), 
//		.h2fReady	     (i2c_h2fReady),
//		.f2hData	     (i2c_f2hData), 
//		.f2hValid      (i2c_f2hValid),
//		.f2hReady      (i2c_f2hReady),
//        .ready         (ready),
//		.SCL	          (SCL),
//		.SDA           (SDA)
//	);

// Dependencies: http://opencores.org/project,i2c
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module CommFPGA_I2Cmaster #(parameter[6:0] BASEADDR = 0, parameter[15:0] PRER = 16'hffff)(
	input wire fx2Clk,
	input wire reset,
	 
	input wire [6:0] chanAddr,
	input wire [7:0] h2fData,
	input wire h2fValid,
	output wire h2fReady,
	output wire [7:0] f2hData,
	output wire f2hValid,
	input wire f2hReady,
     output wire ready,
	inout wire SCL,
	inout wire SDA
	);

	//FPGALink Connections
	//Address allocation logic
	//This logic allocates commFPGA chanAddr addresses to the mux channels
	wire[7:0] muxAddr;
	assign muxAddr	=	(chanAddr < BASEADDR)		?	8'd0			:		//chanAddr out of range low
					(chanAddr == BASEADDR)		?	8'd1			:		//chanAddr 0 map to FIFO
					(chanAddr < BASEADDR + 4)	?	8'd2			:		//addr 1, 2, 3 map to the status registers
					(chanAddr < BASEADDR + 6)	?	8'd3			:		//addr 4, 5 maps to config registers
					 8'd0;											//chanAddr out of range high
									
	wire[6:0] FIFO_chanAddr;
	wire[7:0] FIFO_h2fData;
	wire FIFO_h2fValid;
	wire FIFO_h2fReady;
	
	wire[7:0] FIFO_f2hData;
	wire FIFO_f2hValid;
	wire FIFO_f2hReady;									

	wire[6:0] stat_chanAddr;
	wire[7:0] stat_h2fData;
	wire stat_h2fValid;
	wire stat_h2fReady;
	
	wire[7:0] stat_f2hData;
	wire stat_f2hValid;
	wire stat_f2hReady;
	
	wire[6:0] config_chanAddr;
	wire[7:0] config_h2fData;
	wire config_h2fValid;
	wire config_h2fReady;
	
	wire[7:0] config_f2hData;
	wire config_f2hValid;
	wire config_f2hReady;
	
	commFPGA_mux #(.NMUX(3)) FPGALink_mux (
		.mux				(muxAddr),
		
		//input from commFPGA module
		.chanAddr		(chanAddr),
		.h2fData		(h2fData),
		.h2fValid		(h2fValid),
		.h2fReady		(h2fReady),
		.f2hData		(f2hData),
		.f2hValid		(f2hValid),
		.f2hReady		(f2hReady),
		
		//muxed output to application
		.chanAddr_mux	({config_chanAddr, stat_chanAddr, FIFO_chanAddr}),
		.h2fData_mux	({config_h2fData, stat_h2fData, FIFO_h2fData}),
		.h2fValid_mux	({config_h2fValid, stat_h2fValid, FIFO_h2fValid}),
		.h2fReady_mux	({config_h2fReady, stat_h2fReady, FIFO_h2fReady}),
		.f2hData_mux	({config_f2hData, stat_f2hData, FIFO_f2hData}),
		.f2hValid_mux	({config_f2hValid, stat_f2hValid, FIFO_f2hValid}),
		.f2hReady_mux	({config_f2hReady, stat_f2hReady, FIFO_f2hReady})
	);

	wire[7:0] FIFO_Din;
	wire[7:0] FIFO_Dout;
	wire FIFO_rd;
	wire FIFO_wr;
	wire[7:0] FIFO_inCount;
	wire[7:0] FIFO_outCount;
	wire FIFO_empty;
	wire FIFO_full;
	wire core_rst;
	
	CommFPGA_FIFOwrapper8 fifo(
		.fx2Clk		(fx2Clk),
		.reset		(core_rst),				//fifo has to reset if i2c core resets to properly abort transmissions
	 
//		.chanAddr	(FIFO_chanAddr),
		.h2fData	(FIFO_h2fData),
		.h2fValid	(FIFO_h2fValid),
		.h2fReady	(FIFO_h2fReady),
		.f2hData	(FIFO_f2hData),
		.f2hValid	(FIFO_f2hValid),
		.f2hReady	(FIFO_f2hReady),
		
		.rd			(FIFO_rd),
		.wr			(FIFO_wr),
		.Din			(FIFO_Din),
		.Dout		(FIFO_Dout),
		.inCount		(FIFO_inCount),
		.outCount		(FIFO_outCount),
		.full		(FIFO_full),
		.empty		(FIFO_empty) 
	);
	
	//status memory is at BASEADDR + 1
	wire [7:0] Status_flags;
	
	status_memory #(.NREG(3), .NBYTES(1), .BASEADDR(BASEADDR + 1)) StatMem (
			.fx2Clk		(fx2Clk), 
			.reset		(reset), 
			.chanAddr		(stat_chanAddr),
//			.h2fData		(stat_h2fData), 
//			.h2fValid		(stat_h2fValid), 
			.h2fReady		(stat_h2fReady),
			.f2hData		(stat_f2hData), 
			.f2hValid		(stat_f2hValid),
			.f2hReady		(stat_f2hReady), 
			.PDin		({Status_flags, FIFO_outCount, FIFO_inCount}) 
	);

	//config memory is at BASEADDR + 4
	wire [9:0] SlaveAddr;
	wire TenBit;
	wire [4:0] dummy;
	wire [15:0] Bytes2Read;
	wire ch2fr;
	
	assign config_h2fReady = (Status_flags[0]) ? ch2fr : 1'b0;	//don't allow writes to the address or number of bytes if i2cCore is not idle
	configuration_memory #(.NREG(2), .NBYTES(2), .BASEADDR(BASEADDR + 4)) ConfigMem (
			.fx2Clk		(fx2Clk), 
			.reset		(reset), 
			.chanAddr		(config_chanAddr),
			.h2fData		(config_h2fData), 
			.h2fValid		(config_h2fValid), 
			.h2fReady		(ch2fr),
			.f2hData		(config_f2hData), 
			.f2hValid		(config_f2hValid),
			.f2hReady		(config_f2hReady), 
			.PDout		({Bytes2Read, {dummy, TenBit, SlaveAddr}}) 
	);

	
	//I2C controller state machine
	wire core_en, sta, sto, rd, wr, ack, done, irxack, i2c_busy, i2c_al, al, nak;
	wire[7:0] txr, rxr;
	commFPGA_I2Cmaster_fsm fsm(
		.clk_in		(fx2Clk),
		.reset_in		(reset),
		
		//fifo and config interface
		.slave_addr_in	(SlaveAddr),
		.tenbit_in	(TenBit),
		.bytes2read_in	(Bytes2Read),
		.fifo_rd_out	(FIFO_rd),
		.fifo_wr_out	(FIFO_wr),
		.fifo_full_in	(FIFO_full),
		.fifo_empty_in	(FIFO_empty),
		.fifo_din_out	(FIFO_Din),
		.fifo_dout_in	(FIFO_Dout),
		
		//i2c byte controller interface
		.reset_out	(core_rst),
		.core_en_out	(core_en),
		.start_out	(sta),			
		.stop_out		(sto),
		.read_out		(rd),
		.write_out	(wr),
		.ack_out		(ack),
		.txr_out		(txr),
		.done_in		(done),
		.ack_in		(irxack),
		.rxr_in		(rxr),
		.i2c_busy_in	(i2c_busy),
		.i2c_al_in	(i2c_al),
		
		//status flags
		.idle_out		(ready),
		.AL_out		(al),
		.NAK_out		(nak)
	);

	// 
	assign Status_flags[7:0] = {1'b0, i2c_busy, core_rst, core_en, al, nak, ready};
     
	// hookup byte controller block
	wire sda_pad_o, scl_pad_o, scl_padoen_o, sda_padoen_o;
	assign SCL = (scl_padoen_o) ? 1'bz : scl_pad_o;
	assign SDA = (sda_padoen_o) ? 1'bz : sda_pad_o;
	
	i2c_master_byte_ctrl byte_controller (
		.clk      (fx2Clk),
		.rst      (core_rst),
		.nReset   (1'b1),
		.ena      (core_en),
		.clk_cnt  (PRER),
		.start    (sta),
		.stop     (sto),
		.read     (rd),
		.write    (wr),
		.ack_in   (ack),
		.din      (txr),
		.cmd_ack  (done),
		.ack_out  (irxack),
		.dout     (rxr),
		.i2c_busy (i2c_busy),
		.i2c_al   (i2c_al),
		.scl_i    (SCL),
		.scl_o    (scl_pad_o),
		.scl_oen  (scl_padoen_o ),
		.sda_i    (SDA),
		.sda_o    (sda_pad_o),
		.sda_oen  (sda_padoen_o)
	);

endmodule
