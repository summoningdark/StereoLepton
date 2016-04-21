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
// Create Date:    09:18:59 09/21/2015 
// Design Name: 
// Module Name:    top 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// chanAddr mapping
//
// 0 		PicoController disable. write 1 here to take control with the host computer
// 1			Config register, 8-bit {3'bxxx, Lepton_B enable, Lepton_A enable, LCD_B enable, LCD_A enable, global enable}
// 2..10		Letpon_A, see Lepton.v for description
// 11..19		Lepton_B
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

	localparam NREG = 1;                                   //number of config registers
	localparam REGSIZE = 1;                                //number of bytes in a config register
     localparam LCHANSIZE = 9;                              //Number of chanAddr the Lepton core uses
	localparam L_A_BASEADDR = NREG*REGSIZE + 1;            //First Lepton is after the config reg
	localparam L_B_BASEADDR = L_A_BASEADDR + LCHANSIZE;    //second Lepton is after the first
	
	
     //debugging signals.
     wire [7:0] LA_PD, LB_PD;                               // Packet debug {seg[2:0], normal, n_locked, sdone, packet50, reset}       
     wire [7:0]LA_FD, LB_FD;                                // Framer debug {xxxxxx, blank, reset}
	wire[7:0] LA_config, LB_config;                        //
     wire A_overflow, B_overflow;                           //overflow signal from LCD_driver2
     wire A_frame, B_frame;                                 //50%duty cycle frame toggle from LCD_driver2
     
	//internal signals
	wire reset;                                            //reset signal
	wire[1:0] Button, Button_D, Button_U;                  //pushbutton signals 
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
	wire[6:0] mem_chanAddr, LA_chanAddr, LB_chanAddr;      //muxed DVR interfaces to the individual modules
	wire[7:0] mem_h2fData, LA_h2fData, LB_h2fData;
	wire mem_h2fValid, LA_h2fValid, LB_h2fValid;
	wire mem_h2fReady, LA_h2fReady, LB_h2fReady;
	wire[7:0] mem_f2hData, LA_f2hData, LB_f2hData;
	wire mem_f2hValid, LA_f2hValid, LB_f2hValid;
	wire mem_f2hReady, LA_f2hReady, LB_f2hReady;
	
     wire[REGSIZE*8-1:0] memReg0;                           //parallel connections to configuration memory
     
	wire pvalid_A;                                         //Lepton_A Video output Pixel valid strobe
	wire vsync_A, hsync_A;                                 //Lepton_A vsync and hsync
     wire [23:0] pixel_A;                                   //Lepton_A Video output Pixel data
	wire pvalid_B;                                         //Lepton_B Video output Pixel valid strobe
     wire vsync_B, hsync_B;                                 //Lepton_B vsync and hsync
	wire [23:0] pixel_B;                                   //Lepton_B Video output Pixel data
		
	
	//*******************************signal assignment********************************************
	//resets
	assign reset = !locked_int | !memReg0[0];              //reset on DCM not locked or not global enable
	
	//Address allocation logic
	//This logic allocates commFPGA chanAddr addresses to the mux channels
	assign muxAddr	=	(chanAddr == 7'd0)                           ?    8'd0 :         //out of range low(used by PicoController)
					(chanAddr < L_A_BASEADDR)                    ?    8'd1 :         //config mem at 1
					(chanAddr < L_A_BASEADDR + LCHANSIZE)        ?    8'd2	:         //Lepton_A
					(chanAddr < L_B_BASEADDR + LCHANSIZE)        ?    8'd3	:         //Lepton_B
					 8'd0;                                                           //out of range high
	
	//backlight for displays	
	assign DisplayA_LIGHT = memReg0[1];
	assign DisplayB_LIGHT = memReg0[2];	
	
	//unused lines	for SPI memory
	assign MEM_PWR = 0;
	assign MEM_CS = 1;
	assign MEM_SCK = 0;
	assign MEM_D[0] = 1'bz;
	assign MEM_D[1] = 1'bz;
	assign MEM_D[2] = 1'bz;
	assign MEM_D[3] = 1'bz;
	
	//LEDs
	assign LED = (Button_in[1]) ? {DisplayA_DCX, vsync_A, hsync_A, pvalid_A} : 
                                   LA_PD[3:0];
     
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
	.RST                   (!Button_in[0]),
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

	//************************************external modules***************************************************
	//input switch debouncing
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
	commFPGA_pico_controller #(.ROM_FILE("program_rom_v1.0.mem")) controller (
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
		.interrupt( (|Button_U) | (|Button_D)),
		.interrupt_ack(),
		.sleep(1'b0),
		.port_in({6'd0, Button}), 
		.port_out()
	);

	//mux switches commFPGA interface to multiple modules based on .mux input. .mux = 0 links commFPGA to a null sink(always accepts write, always reads 0)
	//No base address subtraction is done, so mux'd modules should have the proper base adress set.
	commFPGA_mux #(.NMUX(3)) FPGALink_mux (
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
		.chanAddr_mux	({LB_chanAddr, LA_chanAddr, mem_chanAddr}),
		.h2fData_mux	({LB_h2fData, LA_h2fData, mem_h2fData}),
		.h2fValid_mux	({LB_h2fValid, LA_h2fValid, mem_h2fValid}),
		.h2fReady_mux	({LB_h2fReady, LA_h2fReady, mem_h2fReady}),
		.f2hData_mux	({LB_f2hData, LA_f2hData, mem_f2hData}),
		.f2hValid_mux	({LB_f2hValid, LA_f2hValid, mem_f2hValid}),
		.f2hReady_mux	({LB_f2hReady, LA_f2hReady, mem_f2hReady})
	);
	
	configuration_memory #(.NREG(NREG), .NBYTES(REGSIZE), .BASEADDR(1)) ConfigMem (
			.fx2Clk		(fx2Clk), 
			.reset		(!locked_int), 
			.chanAddr		(mem_chanAddr),
			.h2fData		(mem_h2fData), 
			.h2fValid		(mem_h2fValid), 
			.h2fReady		(mem_h2fReady),
			.f2hData		(mem_f2hData), 
			.f2hValid		(mem_f2hValid),
			.f2hReady		(mem_f2hReady), 
			.PDout		(memReg0) //these output lines could be used directly by other logic, but intermediate wires make the code easy to read.
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
		.reset(reset | !memReg0[2]),
          
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
	
	Lepton #(
		.CDIV(CDIV),
          .DELAY(DELAY),
		.BASEADDR(L_A_BASEADDR),
		.VERSION(VERSION)
		) Camera_A (
		.sysclk(fx2Clk), 
		.reset(reset | !memReg0[3]),
		
		.fx2Clk		(fx2Clk),
		.chanAddr		(LA_chanAddr),
		.h2fData		(LA_h2fData),
		.h2fValid		(LA_h2fValid), 
		.h2fReady		(LA_h2fReady),
		.f2hData		(LA_f2hData), 
		.f2hValid		(LA_f2hValid),
		.f2hReady		(LA_f2hReady), 
		
          .configReg0    (LA_config),
          .PacketDebug   (LA_PD),
		.FramerDebug   (LA_FD),
          
		//video output
		.vsync		(vsync_A),               //vertical sync
		.hsync		(hsync_A),               //horizontal sync
		.pvalid		(pvalid_A),              //strobe for each pixel
		.pixel		(pixel_A),               //pixel data {RR,GG,BB} or {10'hxxx, 14'hnnnnn}
    
		//telemetry output
		.tsync		(),                      //strobe at beginning of telemetry data. total of 80 words
		.tvalid		(),                      //strobe for each telemetry data word
		.tdata		(),                      //16-bit telemetry word.

		.L_nReset(Lepton_RST[0]),
		.L_Mclk_en(L_clk_en[0]),
		.L_nPwd(Lepton_PWD[0]),
		.L_Scl(Lepton_SCL[0]), 
		.L_Sda(Lepton_SDA[0]), 
		.L_fsync(Lepton_FSYNC[0]), 
		.L_nCs(Lepton_CS[0]),
		.L_Clk(Lepton_CLK[0]), 
		.L_Miso(Lepton_MISO[0])
	);

	Lepton #(
		.CDIV(CDIV),
          .DELAY(DELAY),
		.BASEADDR(L_B_BASEADDR),
		.VERSION(VERSION)
		) Camera_B (
		.sysclk(fx2Clk), 
		.reset(reset | !memReg0[4]),
		
		.fx2Clk		(fx2Clk),
		.chanAddr		(LB_chanAddr),
		.h2fData		(LB_h2fData),
		.h2fValid		(LB_h2fValid), 
		.h2fReady		(LB_h2fReady),
		.f2hData		(LB_f2hData), 
		.f2hValid		(LB_f2hValid),
		.f2hReady		(LB_f2hReady),
		
          .configReg0    (LB_config),
          .PacketDebug   (LB_PD),
		.FramerDebug   (LB_FD),
		
		//video output
		.vsync		(vsync_B),               //vertical sync
		.hsync		(hsync_B),               //horizontal sync
		.pvalid		(pvalid_B),              //strobe for each pixel
		.pixel		(pixel_B),               //pixel data {RR,GG,BB} or {10'hxxx, 14'hnnnnn}
    
		//telemetry output
		.tsync		(),                      //strobe at beginning of telemetry data. total of 80 words
		.tvalid		(),                      //strobe for each telemetry data word
		.tdata		(),                      //16-bit telemetry word. 

		.L_nReset(Lepton_RST[1]),
		.L_Mclk_en(L_clk_en[1]),
		.L_nPwd(Lepton_PWD[1]),
		.L_Scl(Lepton_SCL[1]), 
		.L_Sda(Lepton_SDA[1]), 
		.L_fsync(Lepton_FSYNC[1]), 
		.L_nCs(Lepton_CS[1]),
		.L_Clk(Lepton_CLK[1]), 
		.L_Miso(Lepton_MISO[1])
	);
endmodule
