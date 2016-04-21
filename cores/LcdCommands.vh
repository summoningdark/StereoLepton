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

// LcdCommands.vh
// If we have not included file before, 
// this symbol _commFPGA_incl_ is not defined.
`ifndef _LcdCommands_incl_
`define _LcdCommands_incl_
// Start of include contents

`define	NOP										8'h00
`define	SOFTWARE_RESET                   8'h01
`define	READ_DISPLAY_ID                  8'h04
`define	READ_DISPLAY_STATUS              8'h09
`define	READ_DISPLAY_POWER_MODE          8'h0A
`define	READ_DISPLAY_MADCTL              8'h0B
`define	READ_DISPLAY_PIXEL_FORMAT        8'h0C
`define	READ_DISPLAY_IMAGE_MODE          8'h0D
`define	READ_DISPLAY_SIGNAL_MODE         8'h0E
`define	READ_DISPLAY_SELF_DIAGNOSTICS    8'h0F
`define	SLEEP_IN                         8'h10
`define	SLEEP_OUT                        8'h11
`define	PARTIAL_MODE_ON                  8'h12
`define	NORMAL_DISPLAY_MODE_ON           8'h13
`define	DISPLAY_INVERSION_OFF            8'h20
`define	DISPLAY_INVERSION_ON             8'h21
`define	GAMMA_SET                        8'h26
`define	DISPLAY_OFF                      8'h28
`define	DISPLAY_ON                       8'h29
`define	COLUMN_ADDRESS_SET               8'h2A
`define	PAGE_ADDRESS_SET                 8'h2B
`define	MEMORY_WRITE                     8'h2C
`define	COLOUR_SET                       8'h2D
`define	MEMORY_READ                      8'h2E
`define	PARTIAL_AREA                     8'h30
`define	VERTICAL_SCROLLING_DEFINITION    8'h33
`define	TEARING_EFFECT_LINE_OFF          8'h34
`define	TEARING_EFFECT_LINE_ON           8'h35
`define	MEMORY_ACCESS_CONTROL            8'h36
`define	VERTICAL_SCROLLING_START_ADDRESS 8'h37
`define	IDLE_MODE_OFF                    8'h38
`define	IDLE_MODE_ON                     8'h39
`define	INTERFACE_PIXEL_FORMAT           8'h3A
`define	READ_ID1                         8'hDA
`define	READ_ID2                         8'hDB
`define	READ_ID3                         8'hDC

`endif  
//_LcdCommands_vh_
