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
// Module Name:    commFPGA_pico_controller2 
// Project Name: 
// Target Devices: 
// Tool versions:
// 
// Description: 
// PicoBlaze based system controller to automate configuration/operation of commFPGA based systems.
// The core implements 2 commFPGA DVR busses, one is intended to be connected to the commFPGA_fx2 wrapper, and the other is for
// connecting to the rest of the system. from the fx2 side there is a single register at chanAddr 0 which controls the core.
// when not in reset, the picoblaze controller runs it's program and has control of the system side DVR interface. the host can reset the core by
// writing 0x01 to the config register. while the picoblaze is in reset, the system side DVR interface is connected directly to the fx2 DVR interface. 
// Because this module uses chanAddr 0, the BASEADDR for the rest of the modules in the system should start with 1.
//
// picoblaze information
// The first 7 port_id's are mapped to special function registers. port_id 7..255 are mapped to RAM
//
// To simulate multiple interrupt vectors, 8 interrupt lines are provided. the 8 signals are latched into the Interrupt Flags register.
// the 8 bits in the Interrupt Flags register are OR'd and used as the interrupt signal to the picoblaze. It is up to the user to clear the 
// Interrupt Flag from within the ISR by writing the correct value to port_id 1. It is up to the user how to prioritize the interrupt bits.
//
// Communication with the DVR bus is handled by a special interface engine, which has direct access to system RAM. This engine is controlled through
// special function registers. The interface engine is necessary since the picoblaze core is not capable of read/writes to the DVR bus on every clock,
// and some cores require data presented in a continuous burst. The DVRpointer register holds a pointer to RAM where the data for the DVR transaction
// is read/written(they first byte pointed to is the chanAddr, followed by the data bytes for both reads and writes). The DVRcount register holds a 
// count of how many bytes the DVR transaction contains. the DVRctrl register is used to start a transaction and to check the status of the engine.
//
// To do a DVR write, load the chanAddr followed by whatever bytes need to be sent into (contiguous)RAM. Then set DVRpointer to the ram address of 
// the chanAddr. Set DVRcount to the number of data bytes(not including chanAddr). Poll DVRctrl for bits {3:2} = 2'b00. Set bit 0 in DVRCtrl. Poll 
// for bit 2 to become 0.
//
// To do a DVR read, load the chanAddr into RAM where you want to put the data(data will be read into RAM following the chanAddr). Set the DVRpointer
// to the RAM address of the chanAddr. Set DVRcount to the number of bytes to read(not including the chanAddr). Poll DVRctrl for bits {3:2} = 2'b00.
// Set bit 1 in DVRctrl. Poll for DVRctrl bit 3 to become 0. 
//
// Note: DVR read/write transactions will read/write bytes from RAM starting at the DVRpointer. If the read/write goes over the 255 boundary, some bytes 
// will be written to RAM addresses less than 7. This space is not readable since port_id 0..6 are reserved for special function registers. So avoid
// setting up reads/writes that would wrap around the RAM boundary.
//
// DVRctrl bits
//   Bit            function
//   0              start read transaction
//   1              start write transaction
//   2              read in progress    (read only)
//   3              write in progress   (read only)
//   4..7           reserved
//
// The special function registers also allow read access to the program ROM. This is useful for storing constant data such as strings.
// 
//   Memory Map
//	port_id	Read                                              Write
//	0		port_in                                           port_out
//   1         Interrupt Flags                                   Interrupt Flags
//	2		DVR pointer                                       DVR pointer
//	3		DVR Count                                         DVR Count
//	4		DVR Ctrl									DVR Ctrl (only bits [1:0] are writeable)
//	5		ROM low byte								ROM Addr pointer Low byte
//	6		ROM high byte*								ROM Addr pointer high byte
//   7..255    RAM                                               RAM
//
// Note* reading from port_id 6 will automatically increment the ROM access pointer, this allows simple reads of large datasets stored in the ROM.
//
// Dependencies: Xilinx PicoBlaze_v6 core, opbasm generic rom component or equivilant program rom, 8-bit x 256 dp RAM.
// kcpsm6.v 
// picoblaze_rom.vhdl
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module commFPGA_pico_controller2#(
	parameter ROM_FILE = "pico_program.mem",
     parameter ROM_SIZE = 1,                      //1,2,3 = 1k, 2k, 4k (4k not recommended for spartan6)
     parameter INT_VECTOR = 10'h3ff,              //default interrupt vector is last address
     parameter SCRATCH_PAD = 64
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
	input wire[7:0] interrupt,         //to simulate multiple interrupt vectors, 8 lines are provided.
	input wire sleep,
	input wire [7:0] port_in,		//input port read at port_id 0.
	output reg [7:0] port_out		//output port written at port_id 0.
    );
	
     //config register
	reg confr;
	wire confr_next;
	assign confr_next = (fx2_chanAddr == 7'd0 & fx2_h2fValid) 	? fx2_h2fData[0] 	: 
					 confr;

	
	//DVR bus switching, when confr == 1'b1, the picoblaze is held in reset, and the DVR bus is a passthrough
	// when confr == 1'b0, the picoblaze runs and controls the system side DVR interface.
	wire [6:0] pico_chanAddr;
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
	
	
	assign fx2_h2fReady =	(confr) ? h2fReady	: 1'b1;             //when fx2 is not connected, don't block writes
	assign fx2_f2hData	=	(confr) ? f2hData	: 8'h00;            //when fx2 is not connected read 0x00
	assign fx2_f2hValid =	(confr) ? f2hValid	: 1'b1;             //when fx2 is not connectd don't block reads
	
	assign pico_h2fReady =	(confr) ? 1'b0		: h2fReady;
	assign pico_f2hData	=	(confr) ? 8'h00 	: f2hData;
	assign pico_f2hValid =	(confr) ? 1'b0 	: f2hValid;
     
     //internal signals
     wire        write_strobe, kwrite_strobe,  read_strobe;           //picoblaze read and write strobes
     wire	       kcpsm6_reset;                                        //picoblaze reset signal
     wire	       bram_enable;                                         //picoblaze ROM enable
     wire [7:0]  out_port, in_port, port_out_next, port_id;           //picoblaze port connections
     wire [8:0]  addrl_p1;                                            //low byte for ROM access pointer
	wire [4:0]  addrh_p1;                                            //high nibble for ROM access pointer
	wire	[11:0] address, address2;                                   //address to program ROM
	wire	[17:0] instruction, instruction2;                           //instruction from program ROM
     wire        RAMweA, RAMweB;                                      //RAM write enables
     wire [7:0]  RAMdinB, RAMdoutA, RAMdoutB, RAMaddrB_next;          //RAM connections
     wire [8:0]  RAMaddrB_p1;
     reg  [7:0]  RAMaddrB, RAMdoutB_q;
     wire [7:0]  RAMdoutB_q_next;
     wire        DVRstrobe;                                           //DVR state machine write strobe for DVRctrl register
     wire [1:0]  DVRctrl_update;                                      //DVR state machine write data for DVRctrl register
     wire [1:0]  DVRbusy;                                             //DVR busy signals
     
     //special function registers
     reg [7:0] Iflags, DVRpointer, DVRcount, addrl;
     reg [1:0] DVRctrl;
     reg [3:0] addrh;
     wire [7:0] Iflags_next, DVRpointer_next, DVRcount_next, addrl_next;
     wire [1:0] DVRctrl_next;
     wire [3:0] addrh_next;
	
     assign port_out_next =   (confr)                                           ?    8'd0           :
                              (port_id == 8'd0 & write_strobe)                  ?    out_port       :
                              (port_id[3:0] == 4'd0 & kwrite_strobe)            ?    out_port       :
                               port_out;
     assign Iflags_next =     (confr)                                           ?    8'd0           :
                              (port_id == 8'd1 & write_strobe)                  ?    out_port       :
                              (port_id[3:0] == 4'd1 & kwrite_strobe)            ?    out_port       :
                               Iflags | interrupt;
     assign DVRpointer_next = (confr)                                           ?    8'd0           :
                              (port_id == 8'd2 & write_strobe)                  ?    out_port       :
                              (port_id[3:0] == 4'd2 & kwrite_strobe)            ?    out_port       :
                               DVRpointer;
     assign DVRcount_next =   (confr)                                           ?    8'd0           :
                              (port_id == 8'd3 & write_strobe)                  ?    out_port       :
                              (port_id[3:0] == 4'd3 & kwrite_strobe)            ?    out_port       :
                               DVRcount;
     assign DVRctrl_next =    (confr)                                           ?    2'd0           :
                              (port_id == 8'd4 & write_strobe)                  ?    out_port[1:0]  :
                              (port_id[3:0] == 4'd4 & kwrite_strobe)            ?    out_port[1:0]  :
                              (DVRstrobe)                                       ?    DVRctrl_update :
                               DVRctrl;
     assign addrl_p1 = addrl + 8'd1;
     assign addrl_next =      (confr)                                           ?    8'd0           :
                              (port_id == 8'd5 & write_strobe)                  ?    out_port       :
                              (port_id[3:0] == 4'd5 & kwrite_strobe)            ?    out_port       :
                              (port_id == 8'd6 & read_strobe)                   ?    addrl_p1[7:0]  : 
                               addrl;
	assign addrh_p1 = addrh + 4'd1;
     assign addrh_next =      (confr)                                           ?    4'd0           :
                              (port_id == 8'd6 & write_strobe)                  ?    out_port[3:0]  :
                              (port_id[3:0] == 4'd6 & kwrite_strobe)            ?    out_port[3:0]  :
                              (port_id == 8'd6 & addrl == 8'hff & read_strobe)  ?    addrh_p1[3:0]  :
                               addrh;
     
     //picoblaze input logic
	assign in_port =    (port_id == 8'd0)   ?    port_in                       :
                         (port_id == 8'd1)   ?    Iflags                        :
                         (port_id == 8'd2)   ?    DVRpointer                    :
                         (port_id == 8'd3)   ?    DVRcount                      :
                         (port_id == 8'd4)   ?    {4'h0, DVRbusy, DVRctrl}      :
                         (port_id == 8'd5)   ?    instruction2[7:0]             :
                         (port_id == 8'd6)   ?    instruction2[15:8]            :
                          RAMdoutA;
     
     
     //DVR state machine
     wire h2f_VR, f2h_VR;                              //true when both Valid and Ready are true
     reg h2f_VR_q;                                     //need a registered version of VR signal to have first-word-fallthrough
     wire RAM_setup;                                   //setup signal from DVR state machine(gets everything ready for RAM to act like a FIFO)
     reg RAM_setup_q;
     assign h2f_VR = pico_h2fValid & pico_h2fReady;    //valid and ready signal
     assign f2h_VR = pico_f2hValid & pico_f2hReady;
     
     
     assign RAMaddrB_p1 =     RAMaddrB + 1;            //incremented RAM address, using only bits [7:0] gives wraparound to 0. 
     assign RAMdoutB_q_next = (h2f_VR_q | h2f_VR | RAM_setup_q) ? RAMdoutB : RAMdoutB_q;       //registered data updates on VR,VRQ or setup_q
     assign RAMaddrB_next =   (RAM_setup)         ? RAMaddrB_p1[7:0] :                         //address increments on setup
                              (DVRbusy[0])        ? (f2h_VR) ? RAMaddrB_p1[7:0] : RAMaddrB :   //during reads, address updates on f2h_VR
                              (DVRbusy[1])        ? (h2f_VR) ? RAMaddrB_p1[7:0] : RAMaddrB :   //during writes, address updates on h2f_VR
                               DVRpointer;
     assign pico_h2fData =    (h2f_VR_q)          ? RAMdoutB : RAMdoutB_q;                     //output data toggles based on VRQ
     
     
     assign RAMweB = f2h_VR;
     assign RAMdinB = pico_f2hData;
     DVRengine DVRinterface(
          .clk_in        (fx2Clk),
          .reset_in      (confr | reset),
          
          //control
          .ctrl_in       (DVRctrl),
          .count_in      (DVRcount),
          .busy_out      (DVRbusy),
          .DVRstrobe_out (DVRstrobe),
          .ctrl_out      (DVRctrl_update),
                   
          //RAM
          .RAM_setup_out (RAM_setup),
          .RAMdout_in    (RAMdoutB),
          
          //DVR interface
          .chanAddr_out  (pico_chanAddr),
          .h2fValid_out  (pico_h2fValid),
          .h2fReady_in   (pico_h2fReady),
          .f2hReady_out  (pico_f2hReady),
          .f2hValid_in   (pico_f2hValid)
     );
     
     //RAM
     //dual ported to give DMA access to the DVR state machine  
     assign RAMweA = (port_id > 6) ? write_strobe : 1'b0;
     
     DPRAM_8x256 RAM(
          .clka(fx2Clk),      // input clka
          .wea(RAMweA),       // input [0 : 0] wea
          .addra(port_id),    // input [7 : 0] addra
          .dina(out_port),    // input [7 : 0] dina
          .douta(RAMdoutA),   // output [7 : 0] douta
          .clkb(fx2Clk),      // input clkb
          .web(RAMweB),       // input [0 : 0] web
          .addrb(RAMaddrB),   // input [7 : 0] addrb
          .dinb(RAMdinB),     // input [7 : 0] dinb
          .doutb(RAMdoutB)    // output [7 : 0] doutb
     );
	
	//picoblaze program memory
	//uses dual ported memory so you can store lots of data in the ROM and have access to it with the picoblaze.
	
	assign address2 = {addrh, addrl};
	
	picoblaze_dp_rom #(
		.ROM_FILE(ROM_FILE))     
	program_rom (
		.Clock				(fx2Clk),
		.Enable				(bram_enable),
		.Address				(address[8+ROM_SIZE:0]),      //size of address determines the size of the infered ROM [9:0] = 1k, [10:0] 2k, [11:0] = 4k
		.Instruction			(instruction),
		.Address2        		(address2[8+ROM_SIZE:0]),     //use the same fraction of address2 as used for address
		.Instruction2    		(instruction2),
		.We					(),				//don't write to the instruction space
		.Wr_instruction2 		()
		);
	assign kcpsm6_reset = reset | confr;
	
	//picoblaze processor
	kcpsm6 #(
		.interrupt_vector		(INT_VECTOR),
		.scratch_pad_memory_size	(SCRATCH_PAD),
		.hwbuild				(8'h00))
	processor (
		.address 				(address),
		.instruction 			(instruction),
		.bram_enable 			(bram_enable),
		.port_id 				(port_id),
		.write_strobe 			(write_strobe),
		.k_write_strobe 		(kwrite_strobe),
		.out_port 			(out_port),
		.read_strobe 			(read_strobe),
		.in_port 				(in_port),
		.interrupt 			(|Iflags),
		.interrupt_ack 		(),
		.reset 				(kcpsm6_reset),
		.sleep				(sleep),
		.clk 				(fx2Clk)); 
      
      
     //registers 
	always @(posedge fx2Clk) begin
          if (reset) begin
               confr <= 1'b0;
               port_out <= 8'h00;
               Iflags <= 8'h00;
               DVRpointer <= 8'h00;
               DVRcount <= 8'h00;
               DVRctrl <= 2'b00;
               addrl <= 8'h00;
               addrh <= 4'h0;
               h2f_VR_q <= 1'b0;
               RAMaddrB <= 8'h00;
               RAMdoutB_q <= 8'h00;
               RAM_setup_q <= 1'b0;
          end else begin
               confr <= confr_next;
               port_out <= port_out_next;
               Iflags <= Iflags_next;
               DVRpointer <= DVRpointer_next;
               DVRcount <= DVRcount_next;
               DVRctrl <= DVRctrl_next;
               addrl <= addrl_next;
               addrh <= addrh_next;
               h2f_VR_q <= h2f_VR;
               RAMaddrB <= RAMaddrB_next;
               RAMdoutB_q <= RAMdoutB_q_next;
               RAM_setup_q <= RAM_setup;
          end
	end
endmodule
