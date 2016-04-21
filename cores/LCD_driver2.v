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
// Create Date:    20:32:45 02/11/2016 
// Design Name: 
// Module Name:    LCD_driver2 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: input video must have at least 1 sysclk from vsync/hsync to first valid pixel.
//
// Dependencies: 24-bit wide x 2048 deep common clock simple dual port RAM. use Xilinx core generator.
//               9-bit by 9-bit multiplier with 1 clk latency.
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`include "LcdCommands.vh"

module LCD_driver2#(
		parameter CLKDIV = 3'd2,			//system clock divisor. set so that 8MHz <= sysclk/2(CLKDIV+1) <=10MHz
		parameter ROTATE = 2'd0,           //Display rotation (0,1,2,3)
		parameter HUPSCALE = 2,            //Horizontal upsacle factor (1 is no upscaling)
		parameter VUPSCALE = 2,            //Vertical upscale factor
		parameter[8:0] HOFFSET = 0,        //Horizontal offset in LCD pixels
		parameter[8:0] VOFFSET = 0,        //Vertical offset in LCD pixels
		parameter[8:0] HSIZE = 160,        //Input video horizontal pixels
		parameter[8:0] VSIZE = 120,        //Input video vertical pixels
          parameter RESET_W = 512,           //for simulation, allows setting LCD reset pulse (default for 48MHz sysclk)
          parameter DELAY = 6000000,         //for simulation, allows setting LCD reset delay (default for 48MHz sysclk)
          parameter DELAY2 = 240000          //for simulation, allows setting LCD sleep_out delay (default for 48MHz sysclk)
     )(
     input wire sysclk,                      //system clock
     input wire reset,                       //system reset
     
     //LCD connections
     output wire [7:0] LCD_D,                // Data lines
     output wire wrx,                        // write enable (rising edge latches data) Max 10MHz. should idle high.
     output wire resx,                       // active low reset min 10us duration, wait 5ms before command, 120ms before operation
     output wire dcx,                        // Data/Command select low = command, high = data
     input  wire te,                         // tearing effect signal
     
     //video input
     input wire vsync,                       // strobe to indicate the start of a video frame (1-sysclk wide)
     input wire hsync,                       // strobe to indicate the start of a video line (1-sysclk wide)
     input wire pvalid,                      // strobe to latch pixel data into the internal buffer
     input wire [23:0] pixel,                // pixel data {RR,GG,BB}
     
     //debug outputs
     output reg overflow,                    //indicates that incomming line rate was too fast to be written to the LCD
     output reg frame = 1'b0                 //toggles every te signal from the LCD, used to monitor frame rate and sync.
     );
    
     //first calculate some parameters
     localparam LCDH = getLCDWidth(ROTATE);                                //LCD width depends on rotation setting
     localparam LCDV = getLCDHeight(ROTATE);                               //LCD height depends on rotation setting
     localparam[15:0] FIRSTCOL = Max(LCDH-1, HOFFSET);                     //calculate the first horizontal LCD pixel
     localparam[15:0] FIRSTROW = Max(LCDV-1, VOFFSET);                     //calculate the first vertical LCD pixel
     localparam[15:0] LASTCOL = Max(LCDH-1, HOFFSET + HSIZE*HUPSCALE-1);   //calculate the last horizontal LCD pixel
     localparam[15:0] LASTROW = Max(LCDV-1, VOFFSET + VSIZE*VUPSCALE-1);   //calculate the last vertical LCD pixel
     localparam[8:0] BUFFERPIX = Ceil_N_M((LASTCOL-FIRSTCOL+1), HUPSCALE); //calculate number of input pixesl are actually used.
     localparam[8:0] VIDLINES = Ceil_N_M((LASTROW-FIRSTROW+1), VUPSCALE);  //calculate the number of input video lines actually used.
     localparam[8:0] NBUFFERS = Max(320, 2048/BUFFERPIX);                  //calculate the number of buffers which fit in the pixel memory
     
     function integer getLCDWidth;
     input integer value;
     begin
          if (value == 0 | value == 3)
               getLCDWidth = 240;
          else
               getLCDWidth = 320;
     end
     endfunction

     function integer getLCDHeight;
     input integer value;
     begin
          if (value == 0 | value == 3)
               getLCDHeight = 320;
          else
               getLCDHeight = 240;
     end
     endfunction
     
     function integer Max;
     input integer max, size;
     begin
          if (size > max)
               Max = max;
          else
             Max = size;
     end
     endfunction
         
     function integer Ceil_N_M;
     input integer N, M;
     integer k;
     begin
          k = N/M;
          if(k * M < N)
               k = k + 1;
          Ceil_N_M = k;
     end
     endfunction
     
     //pixel buffer memory
     //simple dual-port ram, 24bit x 2048, common clock (sysclk) with 1 cycle read latency.
     reg [10:0] addr_A;
     reg [23:0] din_A;
     reg wea;
     wire [10:0] addr_B;
     wire [23:0] dout_B;
     
     DPRAM24x2048 buffer (
          .clka(sysclk), // input clka
          .wea(wea),     // input [0 : 0] wea
          .addra(addr_A), // input [10 : 0] addra
          .dina(din_A),   // input [23 : 0] dina
          .clkb(sysclk), // input clkb
          .addrb(addr_B), // input [10 : 0] addrb
          .doutb(dout_B)  // output [23 : 0] doutb
     );
     
     //clock divider to generate LCD_wrx
     wire LCDclk_pedg, LCDclk_nedg, LCDclk_int, wrx_en;
     assign wrx = (wrx_en) ? LCDclk_int : 1'b1;             //allow wrx clock to be disabled by the LCD state machine.
     clock_divider #(
          .NBITS(3), 
          .POL(1)
     )LCDclk(
          .sysclk(sysclk),
          .reset(reset),
          .div(CLKDIV),
          .p_edge(LCDclk_pedg),
          .n_edge(LCDclk_nedg),
          .clk(LCDclk_int)
     );

     //pixel buffering logic
     wire overflow_next;
     wire[8:0] buffnum_next, pixelnum_next;            //buffer and pixel count for memory address generation
     reg[8:0] buffnum, pixelnum;
     wire[8:0] nextbuff, nextpixel;                    //the buffer or pixel to use next (ie buffnum+1 with wrap to 0)
     reg[8:0] filled_buff;                             //buffer number of the most recent fully filled buffer
     wire[8:0] filled_buff_next;
     reg fl;                                           //toggles high on vsync, low fsync
     reg fl2;                                          //toggles high on vsync or hsync, low on b_up
     wire fl2_next;
     wire fl_next;
     reg fsync;                                        //strobe to indicate that the first buffer of a frame is ready.
     wire fsync_next;
     wire[10:0] addra_next;
     wire p_up, b_up;                                  //pixel and buffer count signals
     
     assign fl_next = (vsync) ? 1'b1 :
                      (fsync) ? 1'b0 :
                       fl;
     assign fl2_next = (hsync | vsync) ? 1'b1 :
                       (b_up) ? 1'b0 :
                        fl2;
     assign nextbuff = (buffnum == NBUFFERS-1) ? 0 : buffnum + 1;
     assign nextpixel = (pixelnum == BUFFERPIX) ? 0 : pixelnum + 1;
     assign buffnum_next = (vsync) ? 0 :
                           (b_up) ? nextbuff : buffnum;
     
     assign pixelnum_next = (hsync | vsync) ? 0 :
                            (p_up) ? nextpixel : pixelnum;
     
     assign fsync_next = fl & pixelnum == BUFFERPIX & ! fsync;                  
     assign p_up = pvalid & !(pixelnum == BUFFERPIX);
     assign b_up = (pixelnum == BUFFERPIX) & fl2;
     assign filled_buff_next = (b_up) ? buffnum : filled_buff;
          
     //pixel memory write address generation.
     //write address = buffnum * BUFFERPIX + pixelnum
     wire[17:0] prod_a;
     multiplier input_mult (
     .clk(sysclk),                      // input clk
     .a(buffnum),                       // input [8 : 0] a
     .b(BUFFERPIX),                     // input [8 : 0] b
     .p(prod_a)                         // output [17 : 0] p
     );
     assign addra_next = pixelnum + prod_a[10:0];
     
     always @(posedge sysclk) begin
          if (reset) begin
               buffnum <= 0;
               pixelnum <= 0;
               addr_A <= 0;
               din_A <= 0;
               wea <= 0;
               filled_buff <= 0;
               fsync <= 0;
               fl <= 0;
               fl2 <= 0;
               overflow <= 0;
          end else begin
               buffnum <= buffnum_next;
               pixelnum <= pixelnum_next;
               addr_A <= addra_next;
               din_A <= pixel;
               wea <= p_up;
               filled_buff <= filled_buff_next;
               fsync <= fsync_next;
               fl <= fl_next;
               fl2 <= fl2_next;
               overflow <= overflow_next;
          end
     end
     
     //LCD startup commands ROM
     //need to do some math to calculate the page_address_set and column_address_set values
     //they will depend on HOFFSET,VOFFSET,HSIZE,VSIZE and ROTATE.
     //four ROTATE cases
     reg[8:0] ROM_D;
     wire[4:0] Startup_A;
          always @* begin
               //output is {DCX, Data[7:0]}
               case (Startup_A)
               5'd0:  ROM_D = {1'b0, `DISPLAY_INVERSION_OFF};
               5'd1:  ROM_D = {1'b0, `IDLE_MODE_OFF};
               5'd2:  ROM_D = {1'b0, `NORMAL_DISPLAY_MODE_ON};
               5'd3:  ROM_D = {1'b0, `INTERFACE_PIXEL_FORMAT};
               5'd4:  ROM_D = {1'b1, 8'h77};
               5'd5:  ROM_D = {1'b0, `MEMORY_ACCESS_CONTROL};
               5'd6:  ROM_D = {1'b1, ROTATE, 2'b10, 4'h0};
               5'd7:  ROM_D = {1'b0, `PAGE_ADDRESS_SET};
               5'd8:  ROM_D = {1'b1, FIRSTROW[15:8]};
               5'd9:  ROM_D = {1'b1, FIRSTROW[7:0]};
               5'd10: ROM_D = {1'b1, LASTROW[15:8]};
               5'd11: ROM_D = {1'b1, LASTROW[7:0]};
               5'd12: ROM_D = {1'b0, `COLUMN_ADDRESS_SET};
               5'd13: ROM_D = {1'b1, FIRSTCOL[15:8]};
               5'd14: ROM_D = {1'b1, FIRSTCOL[7:0]};
               5'd15: ROM_D = {1'b1, LASTCOL[15:8]};
               5'd16: ROM_D = {1'b1, LASTCOL[7:0]};
               5'd17: ROM_D = {1'b0, `TEARING_EFFECT_LINE_ON};
               5'd18: ROM_D = {1'b1, 8'h00};
               5'd19: ROM_D = {1'b0, `DISPLAY_ON};
               default: ROM_D = 0;
               endcase
          end
    
     //LCD pixel output state machine
     wire[8:0] buffnum_b, pixelnum_b;
     LCD_output #(
          .COUNT_W(23),                 //number of bits in the delay counter. must be enough to count a 120ms delay with sysclk
          .RESET_W(RESET_W),            //reset pulse width in sysclk. must be >= 10us
          .DELAY(DELAY),                //delay after reset pulse in sysclk. must be >=120ms
          .DELAY2(DELAY2),              //delay after sleep out command. must be >= 5ms
          .HUPSCALE(HUPSCALE),          //horizontal upscaling factor. input pixels will be duplicated this many times horizontally
          .VUPSCALE(VUPSCALE),          //vertical upscaling factor. input pixels will be duplicated this many times vertically
          .HSIZE(BUFFERPIX),            //number of pixels to output per line
          .VSIZE(VIDLINES),             //number of lines to output
          .NBUFFERS(NBUFFERS)           //number of buffers available
     )output_fsm(
     .clk_in        (sysclk),		     //system clock
     .reset_in      (reset),            //reset			
     .Pedg_in       (LCDclk_pedg),		//posedge strobe for wrx			
     .Nedg_in       (LCDclk_nedg),		//negedge strobe for wrx
     .wrx_en_out    (wrx_en),           //LCD write clock enable
     
     .filledbuff_in (filled_buff),      //input for most recent filled buffer number.
     .busybuff_in   (buffnum),          //input for buffer currently being filled
     .fsync_in      (fsync),            //strobe to start a frame
          
     .startup_a_out (Startup_A),        //LCD startup commands ROM address
     .startup_d_in  (ROM_D),            //LCD startup commands ROM data
     
     .nbuff_out     (buffnum_b),        //buffer number for RAM address generation
     .npixel_out    (pixelnum_b),       //pixel number for RAM address generation
     .pixel_in      (dout_B),           //pixel data from RAM
     
     .data_out      (LCD_D),            //LCD data signals
     .resx_out      (resx),             //LCD reset
     .dcx_out       (dcx)               //LCD data/command signal
     );
     
     //pixel memory read address generation.
     //read address = buffnum * BUFFERPIX + pixelnum
     wire[17:0] prod_b;
     multiplier output_mult (
     .clk(sysclk),                      // input clk
     .a(buffnum_b),                     // input [8 : 0] a
     .b(BUFFERPIX),                     // input [8 : 0] b
     .p(prod_b)                         // output [17 : 0] p
     );
     assign addr_B = (reset) ? 0 : pixelnum_b + prod_b[10:0];
     
     assign overflow_next = (b_up) ? nextbuff == buffnum_b : overflow;
     
     //te edge detect and frame toggle
     reg[1:0] te_q;
     always @(posedge sysclk) begin
          te_q[0] <= te;
          te_q[1] <= te_q[0];
          if (te_q[0] & !te_q[1]) frame <= !frame;
     end
endmodule
