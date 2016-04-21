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
// Engineer: 		Jennifer Holt
// 
// Create Date:    20:55:37 11/05/2015 
// Design Name: 
// Module Name:    commFPGA_pico_controller 
// Project Name: 
// Target Devices: 
// Tool versions:
// 
// Description: 
// PicoBlaze based system controller to automate configuration/operation of commFPGA based systems.
// There core implements 2 commFPGA DVR busses, one is intended to be connected to the commFPGA_fx2 wrapper, and the other is for
// connectiong to the rest of the system. from the fx2 side there is a single register at chanAddr 0 which controls the core.
// when not in reset, the picoblaze controller runs it's program and has control of the system side DVR interface. the host can reset the core by
// writing 0x01 to the config register. while the picoblaze is in reset, the system side DVR interface is connected directly to the fx2 DVR interface. 
// Because this module uses chanAddr 0, the BASEADDR for the rest of the modules in the system should start with 1.
//
// picoblaze information
// the chanAddr presented to the rest of the system by the picoblaze is in a register at port_id 0.
// reading from port_id 1 allows the program to check the status of h2fReady (ie, for checking if the I2CMaster core is idle)
// writing to port_id 1 sets the write mode. write are buffered in a FIFO since the picoblaze can't keep up with the DVR interface.
// if write mode is 0, all bytes are buffered and then sent in a continuous burst. if write mode is 1, bytes are sent as soon as they are 
// written to the fifo. however since the picoblaze is slow, h2fValid will pulse on and off. Some cores (configuration_memory) expect
// contiguous writes.
// reading from 2 allows the program to check the status of the write fifo. writing to 2 puts bytes in the write fifo.
// reading is done by requesting a number of bytes by writing to the read count register at port_id 3. A write to rdCount also resets
// the read FIFO. Then requested number of bytes will be read into the read FIFO. The read count register can then be polled until it is zero, 
// indicating that the read is complete. the bytes in the read fifo can be read at any time at port_id 4.
// the picoblaze program rom can be read by setting the ROM pointer high and low bytes at 6,5.
// once the pointer is set, it takes a single instruction for the data to appear at the input. the data stored in the rom can be read from
// 5 and 6. reads from 5 return instruction[7:0], reads from 6 return instruction[15:8]. a read from 6 also increments the ROM pointer, however
// it still takes the ROM one instruction cycle to catch up after the ROM pointer change.
// port_id 7 is used for reading and writing the external ports.
// 
//	port_id	Read										Write
//	0		pico_chanAddr								pico_chanAddr
//	1		7'd0, pico_h2fReady							write mode bit, 0x00 = burst, 0x01 = one-by-one
//	2		{6'd0, wrFIFO_full, wrFIFO_empty}				wrFIFO Data
//	3		rdCount									rdCount
//	4		rdFIFO Data								----
//	5		ROM low byte								ROM Addr Low byte
//	6		ROM high byte*								ROM Addr high byte
//	7		port_in									port_out
//
// Note* reading from port_id 6 will automatically increment the ROM access pointer, this allows simple reads of large datasets stored in the ROM.
//
// Dependencies: Xilinx PicoBlaze core, opbasm generic rom component or equiviland program rom, 8-bit wide fifo.
// kcpsm6.v 
// picoblaze_rom.vhdl
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module commFPGA_pico_controller#(
	parameter ROM_FILE = "pico_program.mem"
	)(
	input wire fx2Clk,
	input wire reset,
	
	
	//input DVR bus (connect to commFPGA_fx2wrapper)
	input wire [6:0]	fx2_chanAddr,
	input wire [7:0]	fx2_h2fData,
	input wire 		fx2_h2fValid,
	output wire		fx2_h2fReady,
	output wire [7:0]	fx2_f2hData,
	output wire		fx2_f2hValid,
	input wire		fx2_f2hReady,
	
	//output DVR bus (connect to rest of the system, usually a commFPGA_mux)
	output wire[6:0]	chanAddr,
	output wire[7:0]	h2fData,
	output wire		h2fValid,
	input wire		h2fReady,
	input wire[7:0]	f2hData,
	output wire		f2hReady,
	input wire		f2hValid,
	
	//picoblaze specific connections
	input wire interrupt,
	output wire interrupt_ack,
	input wire sleep,
	input wire [7:0] port_in,		//input port read at port_id 7.
	output reg [7:0] port_out		//output port written at port_id 7.
    );
	wire [7:0] port_out_next;
	
	//config register
	reg confr;
	wire confr_next;
	assign confr_next = (reset) 							? 1'b0			:
					(fx2_chanAddr == 7'd0 & fx2_h2fValid) 	? fx2_h2fData[0] 	: 
					 confr;

	
	//DVR bus switching, when confr == 1'b1, the picoblaze is held in reset, and the DVR bus is a passthrough
	// when confr == 1'b0, the picoblaze runs and controls the system side DVR interface.
	reg [6:0] pico_chanAddr;
	wire [6:0] pico_chanAddr_next;
	wire [7:0] pico_h2fData;
	wire pico_h2fValid;
	wire pico_h2fReady;
	wire [7:0] pico_f2hData;
	wire pico_f2hValid;
	wire pico_f2hReady;
	
	assign chanAddr =	(confr) 	? fx2_chanAddr : pico_chanAddr;
	assign h2fData	 =	(confr)	? fx2_h2fData  : pico_h2fData;
	assign h2fValid =	(confr)	? fx2_h2fValid : pico_h2fValid;
	assign f2hReady =	(confr)	? fx2_f2hReady : pico_f2hReady;
	
	
	assign fx2_h2fReady =	(confr) ? h2fReady	: 1'b1;			//when fx2 is not connected, don't block writes
	assign fx2_f2hData	=	(confr) ? f2hData	: 8'h00;			//when fx2 is not connected read 0x00
	assign fx2_f2hValid =	(confr) ? f2hValid	: 1'b1;			//when fx2 is not connectd don't block reads
	
	assign pico_h2fReady =	(confr) ? 1'b0		: h2fReady;
	assign pico_f2hData	=	(confr) ? 8'h00 	: f2hData;
	assign pico_f2hValid =	(confr) ? 1'b0 	: f2hValid;
	
	
	//DVR to picoblaze interface
	reg [7:0] rdCount;
	wire [7:0] rdCount_next;
	reg wr_mode;
	wire wr_mode_next;
	
	wire rdFIFO_full;
	wire [7:0] rdFIFO_dout;
	wire rdFIFO_rd;
	wire rdFIFO_wr;
	
	wire wrFIFO_wr;
	wire wrFIFO_rd;
	wire wrFIFO_full;
	wire wrFIFO_empty;
	
	wire	[7:0]	out_port;
	wire [7:0]	port_id;
	wire	[7:0]	in_port;
	wire			write_strobe;
	wire 		wrk_strobe;
	wire 		read_strobe;
	
	//FIFOs
	CCfifo8_fallthrough rdFIFO(
		.clk(fx2Clk), 		// input clk
		.rst(reset | (port_id == 8'd3 & write_strobe)), 		// input rst
		.din(pico_f2hData), // input [7 : 0] din
		.wr_en(rdFIFO_wr), 	// input wr_en
		.rd_en(rdFIFO_rd),	// input rd_en
		.dout(rdFIFO_dout),	// output [7 : 0] dout
		.full(rdFIFO_full), // output full
		.empty(), 		// output empty
		.data_count()
	);
	
	CCfifo8_fallthrough wrFIFO(
		.clk(fx2Clk), 		// input clk
		.rst(reset), 		// input rst
		.din(out_port), 	// input [7 : 0] din
		.wr_en(wrFIFO_wr), 	// input wr_en
		.rd_en(wrFIFO_rd),	// input rd_en
		.dout(pico_h2fData),// output [7 : 0] dout
		.full(wrFIFO_full), // output full
		.empty(wrFIFO_empty), // output empty
		.data_count()
	);
	
	//picoblaze program memory
	//uses dual ported memory so you can store lots of data in the ROM and have access to it with the picoblaze.
	wire	[11:0]	address, address2;
	reg [7:0] 	addrl; 
	reg [3:0]		addrh;
	wire [7:0]	addrl_next;
	wire [3:0]	addrh_next;
	wire	[17:0]	instruction, instruction2;
	wire			bram_enable;
	wire			kcpsm6_reset;
	
	assign address2 = {addrh, addrl};
	
	picoblaze_dp_rom #(
		.ROM_FILE(ROM_FILE))     
	program_rom (
		.Clock				(fx2Clk),
		.Enable				(bram_enable),
		.Address				(address[9:0]),	//size of address determines the size of the infered ROM [9:0] = 1k, [10:0] 2k, [11:0] = 4k
		.Instruction			(instruction),
		.Address2        		(address2[9:0]),	//use the same fraction of address2 as used for address
		.Instruction2    		(instruction2),
		.We					(),				//don't write to the instruction space
		.Wr_instruction2 		()
		);
	assign kcpsm6_reset = reset | confr;
	
	//picoblaze processor
	kcpsm6 #(
		.interrupt_vector		(12'h3FF),
		.scratch_pad_memory_size	(64),
		.hwbuild				(8'h00))
	processor (
		.address 				(address),
		.instruction 			(instruction),
		.bram_enable 			(bram_enable),
		.port_id 				(port_id),
		.write_strobe 			(write_strobe),
		.k_write_strobe 		(wrk_strobe),
		.out_port 			(out_port),
		.read_strobe 			(read_strobe),
		.in_port 				(in_port),
		.interrupt 			(interrupt),
		.interrupt_ack 		(interrupt_ack),
		.reset 				(kcpsm6_reset),
		.sleep				(sleep),
		.clk 				(fx2Clk)); 

	//picoblaze input logic
	assign in_port =	(port_id == 8'd0)	?	{1'b0, pico_chanAddr}					:
					(port_id == 8'd1)	?	{7'd0, pico_h2fReady}					:
					(port_id == 8'd2)	?	{6'd0, wrFIFO_full, wrFIFO_empty}			:
					(port_id == 8'd3)	?	rdCount								:
					(port_id == 8'd4)	?	rdFIFO_dout							:
					(port_id == 8'd5)	?	instruction2[7:0]						:
					(port_id == 8'd6)	?	instruction2[15:8]						:
					(port_id == 8'd7) 	?	port_in								:
					 8'hxx;

	//output port logic
	assign port_out_next = (reset)						?	8'h00	:
					   (port_id == 8'd7 & write_strobe)	?	out_port	:
					    port_out;

	//Rom reading logic
	wire [8:0] addrl_p1;
	assign addrl_p1 = addrl + 8'd1;
	wire [4:0] addrh_p1;
	assign addrh_p1 = addrh + 4'd1;
	
	assign addrl_next = (port_id == 8'd5 & write_strobe)				? out_port : 
					(port_id == 8'd6 & read_strobe)				? addrl_p1[7:0] : 
					 addrl;
	assign addrh_next = (port_id == 8'd6 & write_strobe) 				? out_port[3:0] : 
					(port_id == 8'd6 & addrl == 8'hff & read_strobe)	? addrh_p1[3:0] :
					 addrh;
	
	//DVR read logic
	assign pico_f2hReady = rdCount > 8'd0 & !rdFIFO_full;
	assign rdFIFO_rd = (port_id == 8'd4) ? read_strobe : 1'b0;
	assign rdFIFO_wr = (pico_f2hValid & pico_f2hReady);
	
	//DVR write logic
	assign pico_h2fValid = !wrFIFO_empty & wr_mode;
	assign wrFIFO_wr = (port_id == 8'd2) ? write_strobe : 1'b0;
	assign wrFIFO_rd = (pico_h2fReady & pico_h2fValid);
	
	wire [7:0] rdCount_m1;
	assign rdCount_m1 = rdCount - 8'd1;
	
	assign pico_chanAddr_next =	(kcpsm6_reset) 					? 7'h00			:
							(port_id == 8'd0 & write_strobe)		? out_port[6:0]	:
							(port_id[3:0] == 4'd0 &wrk_strobe)		? out_port[6:0]	:
							 pico_chanAddr;

	assign rdCount_next = 		(kcpsm6_reset) 				? 8'h00			:
							(port_id == 8'd3 & write_strobe) 	? out_port 		:
							(rdFIFO_wr)					? rdCount_m1 		:
							 rdCount;
								
	assign wr_mode_next =		(kcpsm6_reset)						?	1'b0			:
							(port_id == 8'd1 & write_strobe)		?	out_port[0]	:
							(port_id[3:0] == 4'd1 & wrk_strobe)	?	out_port[0]	:
							 wr_mode;
	
	always @(posedge fx2Clk) begin
		confr <= confr_next;
		pico_chanAddr <= pico_chanAddr_next;
		rdCount <= rdCount_next;
		addrl <= addrl_next;
		addrh <= addrh_next;
		port_out <= port_out_next;
		wr_mode <= wr_mode_next;
	end
endmodule
