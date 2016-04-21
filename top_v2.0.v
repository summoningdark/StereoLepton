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
// Create Date: 
// Design Name: 
// Module Name:    top 
// Project Name:   Stereo Lepton V2.0 
// Target Devices: 
// Tool versions: 
// Description: 
//
// chanAddr mapping
//
// 0 		PicoController disable. write 1 here to take control with the host computer
// 1..2		Config registers, 8-bit see below.
// 3..8		I2C master
//
// Config Register 0
// bit 0:      Global reset, active low(0).
// bit 1:      LCD enable (both A and B)
// bit 2:      Lepton Startup sequence enable (both A & B)
// bit 3:      Lepton Packet enable (both A & B)
// bit 4:      Lepton Framer enable (both A & B)
// bit 5:      I2C mux, 00 = None, 01 = Lepton A, 10 = Lepton B, 11 = Both (can only read from one at a time, can write both)
// bit 6:      I2C mux
// bit 7:      reserved
// Config Register 1 (controls Lepton data interface, does not set values in camera)
// bit 0:      fpixel, pixel format 0= raw14, 1= RGB888     
// bit 1:      Telemetry mod bit0   2'b00 = disabled, 2'b01 = footer, 2'b10 = header, 2'b11 = reserved
// bit 2:      Telemetry mode bit1
// bit 3:      AGC mode, 0= AGC off, 1=AGC on
// bit 4:      reserved
// bit 5:      reserved
// bit 6:      reserved
// bit 7:      reserved
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
//`define LEPTON3
module top
     (
		
		// FX2 interface -----------------------------------------------------------------------------
		input  wire		fx2Clk_in,                    // 48MHz clock from FX2
		output wire		fx2Addr_out1,
		inout  wire[7:0]	fx2Data_io,                   // 8-bit data to/from FX2

		// When EP6OUT selected:
		output wire		fx2Read_out,                  // asserted (active-low) when reading from FX2
		input  wire		fx2GotData_in,                // asserted (active-high) when FX2 has data for us

		// When EP8IN selected:
		output wire		fx2Write_out,                 // asserted (active-low) when writing to FX2
		input  wire		fx2GotRoom_in,                // asserted (active-high) when FX2 has room for more data from us
		output wire		fx2PktEnd_out,                // asserted (active-low) when a host read needs to be committed early
		
		//Lepton interface
		inout  wire[1:0] Lepton_SCL,
		inout  wire[1:0] Lepton_SDA,
		output wire[1:0] Lepton_PWD,
		output wire[1:0] Lepton_RST,
		output wire[1:0] Lepton_MCLK,
		output wire[1:0] Lepton_CS,
		output wire[1:0] Lepton_CLK,
		input  wire[1:0] Lepton_MISO,
		input  wire[1:0] Lepton_FSYNC,
		
		//Display interface
		inout  wire[7:0] DisplayA_D,                      //data bus
		output wire DisplayA_DCX,                         //data/command
		output wire DisplayA_RESX,                        //active low reset
		input  wire DisplayA_TE,	                         //Tearing effect, used to sync to display refresh
		output wire DisplayA_WRX,                         //write enable (rising edge)
		output wire DisplayA_LIGHT,                       //PWM backlight brightness

		inout  wire[7:0] DisplayB_D,
		output wire DisplayB_DCX,
		output wire DisplayB_RESX,
		input  wire DisplayB_TE,
		output wire DisplayB_WRX,
		output wire DisplayB_LIGHT,
		
		//SPI EEPROM memory interface
		output wire MEM_PWR,
		output wire MEM_CS,
		output wire MEM_SCK,
		inout  wire[3:0] MEM_D,
		
		//peripherals
		output wire[3:0] LED,                             //LEDs on board
		input wire [1:0] Button_in                        //pushbutton
    );

`ifdef LEPTON3
     localparam VERSION = 1;
     localparam CDIV = 1;               //sets the VOSPI clock 48/(2*(CDIV+1))MHz. = 12MHz for Lepton3
     localparam DELAY = 22'd2400000;    //number of VOSPI clocks to make a delay of >=185ms (.2s*12MHz = 2400000)
     localparam UPSCALE = 2;
     localparam HSIZE = 160;
     localparam VSIZE = 120;
`else
     localparam VERSION = 0;
     localparam CDIV = 5;               //sets the VOSPI clock 48/(2*(CDIV+1))MHz. = 4MHz for Lepton2 (need to match frame readout time to LCD refresh time)
     localparam DELAY = 22'd800000;     //number of VOSPI clocks to make a delay of >=185ms. (.2s*4MHz = 800000) 
     localparam UPSCALE = 4;
     localparam HSIZE = 80;
     localparam VSIZE = 60;
`endif
	
     //debugging signals.
     wire [7:0] LA_PD, LB_PD;                               // Packet debug {seg[2:0], normal, n_locked, sdone, packet50, reset}       
     wire [7:0]LA_FD, LB_FD;                                // Framer debug {xxxxxx, blank, reset}
	wire[7:0] LA_config, LB_config;                        //
     wire A_overflow, B_overflow;                           //overflow signal from LCD_driver2
     wire A_frame, B_frame;                                 //50%duty cycle frame toggle from LCD_driver2
     
	//internal signals
	wire reset;                                            //reset signal
	wire[1:0] Button, Button_D, Button_U;                  //pushbutton signals
     wire[3:0] Button_command;                              //interpreted command from buttons
	wire fx2Clk;                                           //buffered 48MHz USB clock from cypress fx2
	wire Lclk;                                             //buffered 25MHz Lepton clock from PLL
	wire nLclk;                                            //180 degree phase Lepton clock
	wire locked_int;                                       //PLL locked signal
	wire clkfbout;                                         //PLL feedback clock
	wire clkout0, clkout1;                                 //PLL intermediate clock outputs
	wire[1:0] L_clk_en;                                    //enable signals for Lepton clock output
	
	wire[6:0]	fx2_chanAddr, chanAddr;                      // the selected channel (0-127)
	wire[7:0]	fx2_h2fData, h2fData;                        // data lines used when the host writes to a channel
	wire		fx2_h2fValid, h2fValid;                      // '1' means "on the next clock rising edge, please accept the data on h2fData_out"
	wire		fx2_h2fReady, h2fReady;
	wire[7:0]	fx2_f2hData, f2hData;                        // data lines used when the host reads from a channel
	wire		fx2_f2hReady, f2hReady;                      // '1' means "on the next clock rising edge, put your next byte of data on f2hData_in"
	wire		fx2_f2hValid, f2hValid;
	wire		fx2Reset;
	wire		fx2Addr_out0;                                //unused on this board.
	
	wire[7:0] muxAddr;                                     //indicates which module is connected to the CommFPGA interface
	wire[6:0] mem_chanAddr, I2C_chanAddr;                  //muxed DVR interfaces to the individual modules
	wire[7:0] mem_h2fData, I2C_h2fData;
	wire      mem_h2fValid, I2C_h2fValid;
	wire      mem_h2fReady, I2C_h2fReady;
	wire[7:0] mem_f2hData, I2C_f2hData;
	wire      mem_f2hValid, I2C_f2hValid;
	wire      mem_f2hReady, I2C_f2hReady;
	
     wire[7:0] memReg0, memReg1;                            //parallel connections to configuration memory
     
     wire      Scl, Sda;                                    //I2C connections
     
     wire      syncA, validA, discardA, normalA, specialA, n_lockedA;    //signals from Lepton_PacketA to Lepton FramerA
     wire      syncB, validB, discardB, normalB, specialB, n_lockedB;    //signals from Lepton_PacketB to Lepton FramerB
     wire[7:0] d_byteA, d_byteB;                            //data byte from Lepton_Packet to Lepton_Framer
	wire[15:0]idA, idB, crcA, crcB;                        //id and crc from Lepton_Packet to Lepton Framer
     wire[1:0] t_packets;                                   //number of extra packets added by telemetry
     
	wire      pvalid_A;                                    //Lepton_A Video output Pixel valid strobe
	wire      vsync_A, hsync_A;                            //Lepton_A vsync and hsync
     wire[23:0]pixel_A, pixA;                               //Lepton_A Video output Pixel data
	wire      pvalid_B;                                    //Lepton_B Video output Pixel valid strobe
     wire      vsync_B, hsync_B;                            //Lepton_B vsync and hsync
	wire[23:0]pixel_B, pixB;                               //Lepton_B Video output Pixel data
		
	wire      tsyncA, tsyncB;                              //strobe at beginning of telemetry data. total of 80 words
	wire      tvalidA, tvalidB;                            //strobe for each telemetry data word
	wire[15:0]tdataA, tdataB;                              //16-bit telemetry word.
     
	//*******************************signal assignment********************************************
	//resets
	assign reset = !locked_int | !memReg0[0];              //reset on DCM not locked or not global enable
	
	//Address allocation logic
	//This logic allocates commFPGA chanAddr addresses to the mux channels
	assign muxAddr	=	(chanAddr == 7'd0)       ?    8'd0 :         //out of range low(used by PicoController)
					(chanAddr < 7'd3)        ?    8'd1 :         //config mem at 1..2
					(chanAddr < 7'd9)        ?    8'd2	:         //I2C at 3..8
					 8'd0;                                       //out of range high
	
	//backlight for displays	
	assign DisplayA_LIGHT = memReg0[1];
	assign DisplayB_LIGHT = memReg0[2];	
	
     //I2C muxing
     assign Lepton_SCL[0] = (memReg0[5]) ? Scl  : 1'bz;
     assign Lepton_SCL[1] = (memReg0[6]) ? 1'bz : Scl;
     assign Lepton_SDA[0] = (memReg0[5]) ? Sda  : 1'bz;
     assign Lepton_SDA[1] = (memReg0[6]) ? 1'bz : Sda;
     
	//unused lines	for SPI memory
	assign MEM_PWR = 0;
	assign MEM_CS = 1;
	assign MEM_SCK = 0;
	assign MEM_D[0] = 1'bz;
	assign MEM_D[1] = 1'bz;
	assign MEM_D[2] = 1'bz;
	assign MEM_D[3] = 1'bz;
	
	//LEDs
	assign LED = Button_command;
     
	//------------------------------------Clocking Stuff-------------------------------------
	//fx2 clock input buffer
	
	IBUFG fx2Clk_buf
		(.O (fx2Clk),				//USB Clock
		.I (fx2Clk_in));
	
	PLL_BASE
	#(.BANDWIDTH                 ("OPTIMIZED"),
		.CLK_FEEDBACK           ("CLKFBOUT"),
		.COMPENSATION           ("INTERNAL"),
		.DIVCLK_DIVIDE          (2),
		.CLKFBOUT_MULT          (25),
		.CLKFBOUT_PHASE         (0.000),
		.CLKOUT0_DIVIDE         (24),
		.CLKOUT0_PHASE          (0.000),
		.CLKOUT0_DUTY_CYCLE     (0.500),
		.CLKOUT1_DIVIDE         (24),
		.CLKOUT1_PHASE          (180),
		.CLKOUT1_DUTY_CYCLE     (0.500),
		.CLKIN_PERIOD           (20.833),
		.REF_JITTER             (0.010))
	pll_base_inst
		// Output clocks
	(.CLKFBOUT             (clkfbout),
	.CLKOUT0               (clkout0),
	.CLKOUT1               (clkout1),
	.CLKOUT2               (),
	.CLKOUT3               (),
	.CLKOUT4               (),
	.CLKOUT5               (),
	// Status and control signals
	.LOCKED                (locked_int),
	.RST                   (1'b0),
     // Input clock control
	.CLKFBIN               (clkfbout),
	.CLKIN                 (fx2Clk));
	
	//Lepton clock buffer
	BUFG clkout0_buf(
		.O   (Lclk),
		.I   (clkout0)
	);
	//Lepton nclock buffer
	BUFG clkout1_buf(
		.O   (nLclk),
		.I   (clkout1)
	);
	
     //clock forwarding for Lepton clock output
     //async clear latches to sync signals from fx2Clk domain.
   
	ODDR2 #(
      .DDR_ALIGNMENT("NONE"),      // Sets output alignment to "NONE", "C0" or "C1" 
      .INIT(1'b0),                 // Sets initial state of the Q output to 1'b0 or 1'b1
      .SRTYPE("ASYNC")             // Specifies "SYNC" or "ASYNC" set/reset
   ) ODDR2_LA (
      .Q(Lepton_MCLK[0]),          // 1-bit DDR output data
      .C0(Lclk),				// 1-bit clock input
      .C1(nLclk)	,			// 1-bit clock input
      .CE(1'b1),				// 1-bit clock enable input
      .D0(1'b1),				// 1-bit data input (associated with C0)
      .D1(1'b0), 				// 1-bit data input (associated with C1)
      .R(1'b0),	               // 1-bit reset input
      .S(1'b0)					// 1-bit set input
   );
	
	ODDR2 #(
      .DDR_ALIGNMENT("NONE"), // Sets output alignment to "NONE", "C0" or "C1" 
      .INIT(1'b0),    // Sets initial state of the Q output to 1'b0 or 1'b1
      .SRTYPE("ASYNC") // Specifies "SYNC" or "ASYNC" set/reset
   ) ODDR2_LB (
      .Q(Lepton_MCLK[1]),          // 1-bit DDR output data
      .C0(Lclk),				// 1-bit clock input
      .C1(nLclk)	,			// 1-bit clock input
      .CE(1'b1),				// 1-bit clock enable input
      .D0(1'b1),				// 1-bit data input (associated with C0)
      .D1(1'b0), 				// 1-bit data input (associated with C1)
      .R(1'b0),       	     	// 1-bit reset input
      .S(1'b0)					// 1-bit set input
   );

     //************************************ Logic ***************************************************
     //button command decoding
     reg[1:0] Button_q;                                               //delayed Button state
     always @(posedge fx2Clk) begin
          if (reset) Button_q <= 2'b00; else Button_q <= Button;
     end
     assign Button_command[0] = (Button == 2'b00) & (Button_q == 2'b01);        //command 0 on release of button 0
     assign Button_command[1] = (Button == 2'b00) & (Button_q == 2'b10);        //command 1 on release of button 1
     assign Button_command[2] = (Button == 2'b11) & (Button_q == 2'b10);        //command 2 on press of button 0 while button 1 held
     assign Button_command[3] = (Button == 2'b11) & (Button_q == 2'b01);        //command 3 on press of button 1 while button 0 held
     
     //Lepton Startup sequence
	`ifndef SYNTHESIS
		parameter STARTCOUNTEND = 16'd8;
	`else
		parameter STARTCOUNTEND = 16'd15000;		
	`endif
	
	reg [15:0] Start_Count;						     //startup counter, need to wait at least 5000 LClk = 5000 * 1/25MHz = 200us
	assign Lepton_PWD[0] = Start_Count > 1;			     //Power Down de-asserted
     assign Lepton_PWD[1] = Start_Count > 1;			     //Power Down de-asserted
	assign Lepton_RST[0] = Start_Count == STARTCOUNTEND;	//wait 5000 LClk then reset de-asserted
	assign Lepton_RST[1] = Start_Count == STARTCOUNTEND;	//wait 5000 LClk then reset de-asserted
	
	always @(posedge fx2Clk) begin
		if (reset | !memReg0[2]) begin
			Start_Count <= 0;
		end else begin
			if (Start_Count != STARTCOUNTEND) Start_Count <= Start_Count + 1;
			else Start_Count <= Start_Count;
		end
	end

     //output pixel mapping
     assign pixel_A = (memReg1[0]) ?  pixA :
                                      (memReg1[3]) ? {pixA[7:0], pixA[7:0], pixA[7:0]} : {10'd0, pixA[13:0]};
     assign pixel_B = (memReg1[0]) ?  pixB :
                                      (memReg1[3]) ? {pixB[7:0], pixB[7:0], pixB[7:0]} : {10'd0, pixB[13:0]};
                                      
	//************************************external modules***************************************************
	//input switches
	debounce #(.NBITS(17)) sw0(
		.clk	(fx2Clk),
		.I	(Button_in[0]),
		.O	(Button[0]),
		.U	(Button_U[0]),
		.D	(Button_D[0])
	);
	
	debounce #(.NBITS(17)) sw1(
		.clk	(fx2Clk),
		.I	(Button_in[1]),
		.O	(Button[1]),
		.U	(Button_U[1]),
		.D	(Button_D[1])
	);
	
	//CommFPGA Link wrapper
	commFPGA_fx2wrapper FPGALink(
		.fx2Clk		(fx2Clk),

		// FX2LP interface
		.fx2Addr_out	({fx2Addr_out1, fx2Addr_out0}),
		.fx2Data_io	(fx2Data_io),
		.fx2Read_out	(fx2Read_out),
		.fx2OE_out	(),
		.fx2GotData_in	(fx2GotData_in),
		.fx2Write_out	(fx2Write_out),
		.fx2GotRoom_in	(fx2GotRoom_in),
		.fx2PktEnd_out	(fx2PktEnd_out),

		// DVR interface -> Connects to application module
		.fx2Reset		(fx2Reset),
		.chanAddr		(fx2_chanAddr),
		.h2fData		(fx2_h2fData),
		.h2fValid		(fx2_h2fValid),
		.h2fReady		(fx2_h2fReady),
		.f2hData		(fx2_f2hData),
		.f2hValid		(fx2_f2hValid),
		.f2hReady		(fx2_f2hReady)
	);

	//PicoController for automatic start-up
	commFPGA_pico_controller2 #(.ROM_FILE("program_rom_v2.0.mem")) controller (
		.fx2Clk(fx2Clk), 
		.reset(!locked_int),
          
		.fx2_chanAddr(fx2_chanAddr), 
		.fx2_h2fData(fx2_h2fData), 
		.fx2_h2fValid(fx2_h2fValid), 
		.fx2_h2fReady(fx2_h2fReady), 
		.fx2_f2hData(fx2_f2hData), 
		.fx2_f2hValid(fx2_f2hValid), 
		.fx2_f2hReady(fx2_f2hReady),
		
		.chanAddr(chanAddr), 
		.h2fData(h2fData), 
		.h2fValid(h2fValid), 
		.h2fReady(h2fReady), 
		.f2hData(f2hData), 
		.f2hReady(f2hReady), 
		.f2hValid(f2hValid),
          
		.interrupt({4'h0, Button_command}),          //interrupt on the 4 button commands
		.sleep(1'b0),
		.port_in({6'd0, I2CReady, Lepton_RST[0]}),   //allow montioring of I2C core status and, Lepton Startup status
		.port_out()
	);

	//mux switches commFPGA interface to multiple modules based on .mux input. .mux = 0 links commFPGA to a null sink(always accepts write, always reads 0)
	//No base address subtraction is done, so mux'd modules should have the proper base adress set.
	commFPGA_mux #(.NMUX(2)) FPGALink_mux (
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
		.chanAddr_mux	({I2C_chanAddr, mem_chanAddr}),
		.h2fData_mux	({I2C_h2fData,  mem_h2fData}),
		.h2fValid_mux	({I2C_h2fValid, mem_h2fValid}),
		.h2fReady_mux	({I2C_h2fReady, mem_h2fReady}),
		.f2hData_mux	({I2C_f2hData,  mem_f2hData}),
		.f2hValid_mux	({I2C_f2hValid, mem_f2hValid}),
		.f2hReady_mux	({I2C_f2hReady, mem_f2hReady})
	);
	
	configuration_memory #(.NREG(2), .NBYTES(1), .BASEADDR(1)) ConfigMem (
			.fx2Clk		(fx2Clk), 
			.reset		(!locked_int), 
			.chanAddr		(mem_chanAddr),
			.h2fData		(mem_h2fData), 
			.h2fValid		(mem_h2fValid), 
			.h2fReady		(mem_h2fReady),
			.f2hData		(mem_f2hData), 
			.f2hValid		(mem_f2hValid),
			.f2hReady		(mem_f2hReady), 
			.PDout		({memReg1, memReg0}) //these output lines could be used directly by other logic, but intermediate wires make the code easy to read.
	);     
     
	//Displays
     LCD_driver2#(
		.CLKDIV    (3'd2),          //system clock divisor. set so that 8MHz <= sysclk/2(CLKDIV+1) <=10MHz
		.ROTATE    (2'd1),          //Display rotation (0,1,2,3)
		.HUPSCALE  (UPSCALE),       //Horizontal upsacle factor (1 is no upscaling)
		.VUPSCALE  (UPSCALE),       //Vertical upscale factor
		.HSIZE     (HSIZE),         //Horizontal size of input video
		.VSIZE     (VSIZE)          //Vertical size of input video
     ) Display_A (
		.sysclk(fx2Clk), 
		.reset(reset | !memReg0[1]),
          
		.LCD_D(DisplayA_D), 
		.wrx(DisplayA_WRX), 
		.resx(DisplayA_RESX), 
		.dcx(DisplayA_DCX), 
		.te(DisplayA_TE), 

		.vsync(vsync_A), 
		.hsync(hsync_A), 
		.pvalid(pvalid_A), 
		.pixel(pixel_A),
          
		.overflow(A_overflow), 
		.frame(A_frame)
	);

     LCD_driver2#(
		.CLKDIV    (3'd2),          //system clock divisor. set so that 8MHz <= sysclk/2(CLKDIV+1) <=10MHz
		.ROTATE    (2'd2),          //Display rotation (0,1,2,3)
		.HUPSCALE  (UPSCALE),       //Horizontal upsacle factor (1 is no upscaling)
		.VUPSCALE  (UPSCALE),       //Vertical upscale factor
		.HSIZE     (HSIZE),         //Horizontal size of input video
		.VSIZE     (VSIZE)          //Vertical size of input video
     ) Display_B (
		.sysclk(fx2Clk), 
		.reset(reset | !memReg0[1]),
          
		.LCD_D(DisplayB_D), 
		.wrx(DisplayB_WRX), 
		.resx(DisplayB_RESX), 
		.dcx(DisplayB_DCX), 
		.te(DisplayB_TE), 

		.vsync(vsync_B), 
		.hsync(hsync_B), 
		.pvalid(pvalid_B), 
		.pixel(pixel_B),
          
		.overflow(B_overflow), 
		.frame(B_frame)
	);
	
     //I2C master for CCI
	CommFPGA_I2Cmaster #(.BASEADDR(3), .PRER(16'h19)) CCI (
		.fx2Clk	(fx2Clk),
		.reset	(reset),
	
		.chanAddr	(I2C_chanAddr),
		.h2fData	(I2C_h2fData), 
		.h2fValid	(I2C_h2fValid), 
		.h2fReady	(I2C_h2fReady),
		.f2hData	(I2C_f2hData), 
		.f2hValid	(I2C_f2hValid),
		.f2hReady	(I2C_f2hReady),
          .ready    (I2CReady),
		.SCL		(Scl),
		.SDA		(Sda)
	);
     
`ifdef LEPTON3 
     assign t_packets = (memReg1[2:1] ==0 ) ? 2'd0 : 2'd1;    //telemetry adds 1 packet to each segment for Lepton3
 `else
     assign t_packets = (memReg1[2:1] ==0 ) ? 2'd0 : 2'd3;    //telemetry adds 3 packets to each frame for Lepton2
`endif
      
	Lepton_packet #(
		.CDIV(CDIV),
		.DELAY(DELAY)
	)VOSPI_A(
		.sysclk	(fx2Clk), 
		.reset_in	(reset | !memReg0[3] | !Lepton_RST[0]), 
		.fpixel	(memReg1[0]),
          .telemetry(t_packets),
          //debug outputs
          .reset    (),                                     //internal reset signal
		.packet50	(), 
		.sdone    (),
          .seg		(),
          //packet outputs
		.sync	(syncA),     
		.id		(idA), 
		.crc		(crcA), 
		.valid	(validA), 
		.d_byte	(d_byteA),
          .n_ready  (n_lockedA),
		.discard	(discardA),
		.normal	(normalA),
		.special	(specialA),
		.stuck	(),
		.nCs		(Lepton_CS[0]), 
		.sClk	(Lepton_CLK[0]), 
		.Miso	(Lepton_MISO[0])
	);

	Lepton_packet #(
		.CDIV(CDIV),
		.DELAY(DELAY)
	)VOSPI_B(
		.sysclk	(fx2Clk), 
		.reset_in	(reset | !memReg0[3] | !Lepton_RST[1]), 
		.fpixel	(memReg1[0]),
          .telemetry(t_packets),
          //debug outputs
          .reset    (),                                     //internal reset signal
		.packet50	(), 
		.sdone    (),
          .seg		(),
          //packet outputs
		.sync	(syncB),     
		.id		(idB), 
		.crc		(crcB), 
		.valid	(validB), 
		.d_byte	(d_byteB),
          .n_ready  (n_lockedB),
		.discard	(discardB),
		.normal	(normalB),
		.special	(specialB),
		.stuck	(),
		.nCs		(Lepton_CS[1]), 
		.sClk	(Lepton_CLK[1]), 
		.Miso	(Lepton_MISO[1])
	);
	
	//Lepton video framer
	Lepton_framer #(.VERSION(VERSION)) FramerA(
		.sysclk		(fx2Clk),
		.reset		(reset | n_lockedA | !memReg0[4]),		     //reset when the packet interface is not locked
	
	//format inputs
		.fpixel		(memReg1[0]),		     //pixel format 0= raw14, 1= RGB888 (must match settings on camera and psize on Lepton_packet)
		.telemetry	(memReg1[2:1]),		//telemetry mode. 2'b00 = disabled, 2'b01 = footer, 2'b10 = header, 2'b11 = reserved
	
	//inputs from Lepton_packet
		.sync		(syncA),
		.id			(idA),
		.crc			(crcA),
		.valid		(validA),
		.d_byte		(d_byteA),
		.discard		(discardA),
		.normal		(normalA),
		.special		(specialA),
	
	//video output
		.vsync		(vsync_A),                //strobe at the beginning of each full frame
		.hsync		(hsync_A),                //strobe at the beginning of each line(not used)
		.pvalid		(pvalid_A),               //strobe for each pixel
		.pixel		(pixA),                  //pixel data {RR,GG,BB} or {10'hxxx, 14'hnnnnn}
	
	//telemetry output
		.tsync		(tsyncA),                //strobe at beginning of telemetry data
		.tvalid		(tvalidA),               //strobe for each telemetry data word
		.tdata		(tdataA),                //16-bit telemetry word.
		.blank		()
	);                                

	Lepton_framer #(.VERSION(VERSION)) FramerB(
		.sysclk		(fx2Clk),
		.reset		(reset | n_lockedB | !memReg0[4]),		     //reset when the packet interface is not locked
	
	//format inputs
		.fpixel		(memReg1[0]),		//pixel format 0= raw14, 1= RGB888 (must match settings on camera and psize on Lepton_packet)
		.telemetry	(memReg1[2:1]),		//telemetry mode. 2'b00 = disabled, 2'b01 = footer, 2'b10 = header, 2'b11 = reserved
	
	//inputs from Lepton_packet
		.sync		(syncB),
		.id			(idB),
		.crc			(crcB),
		.valid		(validB),
		.d_byte		(d_byteB),
		.discard		(discardB),
		.normal		(normalB),
		.special		(specialB),
	
	//video output
		.vsync		(vsync_B),				//strobe at the beginning of each full frame
		.hsync		(hsync_B),				//strobe at the beginning of each line(not used)
		.pvalid		(pvalid_B),			//strobe for each pixel
		.pixel		(pixB),				//pixel data {RR,GG,BB} or {10'hxxx, 14'hnnnnn}
	
	//telemetry output
		.tsync		(tsyncB),				//strobe at beginning of telemetry data
		.tvalid		(tvalidB),		     //strobe for each telemetry data word
		.tdata		(tdataB),				//16-bit telemetry word.
		.blank		()
	);
endmodule
