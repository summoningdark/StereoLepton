`include "LcdCommands.vh" 


// Created by fizzim.pl version $Revision: 5.0 on 2016:02:28 at 18:38:42 (www.fizzim.com)

module LCD_output 
  #(
    parameter COUNT_W = 23,
    parameter DELAY = 23'd6000000,
    parameter DELAY2 = 23'd250000,
    parameter HSIZE = 320,
    parameter HUPSCALE = 2,
    parameter NBUFFERS = 1,
    parameter RESET_W = 23'd512,
    parameter VSIZE = 240,
    parameter VUPSCALE = 2
  )(
  output reg [7:0] data_out,
  output reg dcx_out,
  output reg [8:0] nbuff_out,
  output reg [8:0] npixel_out,
  output reg resx_out,              // active low
  output reg [4:0] startup_a_out,
  output reg wrx_en_out,
  input wire Nedg_in,
  input wire Pedg_in,
  input wire [8:0] busybuff_in,
  input wire clk_in,
  input wire [8:0] filledbuff_in,
  input wire fsync_in,
  input wire [23:0] pixel_in,
  input wire reset_in,
  input wire [8:0] startup_d_in 
);

  // state bits
  parameter 
  reset        = 5'b00000, 
  blue         = 5'b00001, 
  blue2        = 5'b00010, 
  count_reset  = 5'b00011, 
  count_reset2 = 5'b00100, 
  green        = 5'b00101, 
  green2       = 5'b00110, 
  red          = 5'b00111, 
  red2         = 5'b01000, 
  resx_delay   = 5'b01001, 
  resx_pulse   = 5'b01010, 
  sleep_delay  = 5'b01011, 
  sleep_out    = 5'b01100, 
  start_frame  = 5'b01101, 
  startup_cmd  = 5'b01110, 
  startup_inc  = 5'b01111, 
  sync         = 5'b10000, 
  sync2        = 5'b10001, 
  sync3        = 5'b10010, 
  sync4        = 5'b10011, 
  wait_buffer  = 5'b10100, 
  wait_frame   = 5'b10101; 

  reg [4:0] state;
  reg [4:0] nextstate;
  reg [COUNT_W-1:0] count;
  reg [3:0] hup_count;
  reg [15:0] pixel_q;
  reg [8:0] tbuff_count;
  reg [3:0] vup_count;

  // comb always block
  always @* begin
    nextstate = state; // default to hold value because implied_loopback is set
    case (state)
      reset       : begin
        begin
          nextstate = resx_pulse;
        end
      end
      blue        : begin
        begin
          nextstate = blue2;
        end
      end
      blue2       : begin
        if (Nedg_in & tbuff_count==0) begin
          nextstate = wait_frame;
        end
        else if (Nedg_in & (nbuff_out==busybuff_in & nbuff_out !=filledbuff_in)) begin
          nextstate = wait_buffer;
        end
        else if (Nedg_in) begin
          nextstate = red;
        end
      end
      count_reset : begin
        begin
          nextstate = resx_delay;
        end
      end
      count_reset2: begin
        begin
          nextstate = sleep_delay;
        end
      end
      green       : begin
        begin
          nextstate = green2;
        end
      end
      green2      : begin
        if (Nedg_in) begin
          nextstate = blue;
        end
      end
      red         : begin
        begin
          nextstate = red2;
        end
      end
      red2        : begin
        if (Nedg_in) begin
          nextstate = green;
        end
      end
      resx_delay  : begin
        if (count == DELAY) begin
          nextstate = sync;
        end
      end
      resx_pulse  : begin
        if (count == RESET_W) begin
          nextstate = count_reset;
        end
      end
      sleep_delay : begin
        if (count==DELAY2 & Nedg_in) begin
          nextstate = startup_cmd;
        end
        else if (count == DELAY2 & !Nedg_in) begin
          nextstate = sync2;
        end
      end
      sleep_out   : begin
        if (Pedg_in) begin
          nextstate = count_reset2;
        end
      end
      start_frame : begin
        if (Nedg_in) begin
          nextstate = red;
        end
      end
      startup_cmd : begin
        if (Pedg_in) begin
          nextstate = startup_inc;
        end
      end
      startup_inc : begin
        if (startup_a_out == 21) begin
          nextstate = wait_frame;
        end
        else if (Nedg_in) begin
          nextstate = startup_cmd;
        end
        else begin
          nextstate = sync3;
        end
      end
      sync        : begin
        if (Nedg_in) begin
          nextstate = sleep_out;
        end
      end
      sync2       : begin
        if (Nedg_in) begin
          nextstate = startup_cmd;
        end
      end
      sync3       : begin
        if (Nedg_in) begin
          nextstate = startup_cmd;
        end
      end
      sync4       : begin
        if (Nedg_in) begin
          nextstate = start_frame;
        end
      end
      wait_buffer : begin
        if (Nedg_in & (nbuff_out != busybuff_in | nbuff_out == filledbuff_in)) begin
          nextstate = red;
        end
      end
      wait_frame  : begin
        if (fsync_in & Nedg_in) begin
          nextstate = start_frame;
        end
        else if (fsync_in) begin
          nextstate = sync4;
        end
      end
    endcase
  end

  // Assign reg'd outputs to state bits

  // sequential always block
  always @(posedge clk_in) begin
    if (reset_in)
      state <= reset;
    else
      state <= nextstate;
  end

  // datapath sequential always block
  always @(posedge clk_in) begin
    if (reset_in) begin
      count[COUNT_W-1:0] <= 0;
      data_out[7:0] <= 0;
      dcx_out <= 0;
      hup_count[3:0] <= 0;
      nbuff_out[8:0] <= 0;
      npixel_out[8:0] <= 0;
      pixel_q[15:0] <= 0;
      resx_out <= 1'b1;
      startup_a_out[4:0] <= 0;
      tbuff_count[8:0] <= 0;
      vup_count[3:0] <= 0;
      wrx_en_out <= 1'b1;
    end
    else begin
      count[COUNT_W-1:0] <= count; // default
      data_out[7:0] <= `NOP; // default
      dcx_out <= 1'b0; // default
      hup_count[3:0] <= hup_count; // default
      nbuff_out[8:0] <= nbuff_out; // default
      npixel_out[8:0] <= npixel_out; // default
      pixel_q[15:0] <= pixel_q; // default
      resx_out <= 1'b1; // default
      startup_a_out[4:0] <= startup_a_out; // default
      tbuff_count[8:0] <= tbuff_count; // default
      vup_count[3:0] <= vup_count; // default
      wrx_en_out <= 1'b1; // default
      case (nextstate)
        reset       : begin
          resx_out <= 1'b0;
          wrx_en_out <= 1'b0;
        end
        blue        : begin
          data_out[7:0] <= pixel_q[7:0];
          dcx_out <= 1'b1;
        end
        blue2       : begin
          data_out[7:0] <= data_out;
          dcx_out <= 1'b1;
        end
        count_reset : begin
          count[COUNT_W-1:0] <= 0;
          wrx_en_out <= 1'b0;
        end
        count_reset2: begin
          data_out[7:0] <= `SLEEP_OUT;
          wrx_en_out <= 1'b0;
        end
        green       : begin
          data_out[7:0] <= pixel_q[15:8];
          dcx_out <= 1'b1;
          nbuff_out[8:0] <= (vup_count !=0) ? nbuff_out : (nbuff_out == NBUFFERS-1) ? 0 : nbuff_out + 1;
          npixel_out[8:0] <= (hup_count != 0) ? npixel_out : (npixel_out==HSIZE-1) ? 0 : npixel_out+1;
          tbuff_count[8:0] <= (vup_count==0) ? tbuff_count-1: tbuff_count;
        end
        green2      : begin
          data_out[7:0] <= data_out;
          dcx_out <= 1'b1;
        end
        red         : begin
          data_out[7:0] <= pixel_in[23:16];
          dcx_out <= 1'b1;
          hup_count[3:0] <= (hup_count ==0) ? HUPSCALE-1 : hup_count-1;
          pixel_q[15:0] <= pixel_in[15:0];
          vup_count[3:0] <= (vup_count==0) ? VUPSCALE : (npixel_out==HSIZE-1 & hup_count ==1) ? vup_count-1:vup_count;
        end
        red2        : begin
          data_out[7:0] <= data_out;
          dcx_out <= 1'b1;
        end
        resx_delay  : begin
          count[COUNT_W-1:0] <= count+1;
          wrx_en_out <= 1'b0;
        end
        resx_pulse  : begin
          count[COUNT_W-1:0] <= count+1;
          resx_out <= 1'b0;
          wrx_en_out <= 1'b0;
        end
        sleep_delay : begin
          count[COUNT_W-1:0] <= count+1;
          data_out[7:0] <= `SLEEP_OUT;
          wrx_en_out <= 1'b0;
        end
        sleep_out   : begin
          count[COUNT_W-1:0] <= 0;
          data_out[7:0] <= `SLEEP_OUT;
        end
        start_frame : begin
          data_out[7:0] <= `MEMORY_WRITE;
          hup_count[3:0] <= HUPSCALE;
          nbuff_out[8:0] <= 0;
          npixel_out[8:0] <= 0;
          tbuff_count[8:0] <= VSIZE;
          vup_count[3:0] <= VUPSCALE;
        end
        startup_cmd : begin
          data_out[7:0] <= startup_d_in[7:0];
          dcx_out <= startup_d_in[8];
        end
        startup_inc : begin
          data_out[7:0] <= data_out;
          dcx_out <= dcx_out;
          startup_a_out[4:0] <= startup_a_out+1;
        end
        sync        : begin
          wrx_en_out <= 1'b0;
        end
        sync2       : begin
          wrx_en_out <= 1'b0;
        end
        sync3       : begin
          data_out[7:0] <= data_out;
          dcx_out <= dcx_out;
        end
        wait_buffer : begin
          wrx_en_out <= 1'b0;
        end
      endcase
    end
  end

  // This code allows you to see state names in simulation
  `ifndef SYNTHESIS
  reg [95:0] statename;
  always @* begin
    case (state)
      reset       :
        statename = "reset";
      blue        :
        statename = "blue";
      blue2       :
        statename = "blue2";
      count_reset :
        statename = "count_reset";
      count_reset2:
        statename = "count_reset2";
      green       :
        statename = "green";
      green2      :
        statename = "green2";
      red         :
        statename = "red";
      red2        :
        statename = "red2";
      resx_delay  :
        statename = "resx_delay";
      resx_pulse  :
        statename = "resx_pulse";
      sleep_delay :
        statename = "sleep_delay";
      sleep_out   :
        statename = "sleep_out";
      start_frame :
        statename = "start_frame";
      startup_cmd :
        statename = "startup_cmd";
      startup_inc :
        statename = "startup_inc";
      sync        :
        statename = "sync";
      sync2       :
        statename = "sync2";
      sync3       :
        statename = "sync3";
      sync4       :
        statename = "sync4";
      wait_buffer :
        statename = "wait_buffer";
      wait_frame  :
        statename = "wait_frame";
      default     :
        statename = "XXXXXXXXXXXX";
    endcase
  end
  `endif

endmodule

