
// Created by fizzim.pl version $Revision: 5.0 on 2016:04:12 at 20:03:56 (www.fizzim.com)

module DVRengine (
  output wire DVRstrobe_out,
  output reg RAM_setup_out,
  output wire [1:0] busy_out,
  output reg [6:0] chanAddr_out,
  output reg [1:0] ctrl_out,
  output reg f2hReady_out,
  output wire h2fValid_out,
  input wire [7:0] RAMdout_in,
  input wire clk_in,
  input wire [7:0] count_in,
  input wire [1:0] ctrl_in,
  input wire f2hValid_in,
  input wire h2fReady_in,
  input wire reset_in 
);

  // state bits
  parameter 
  Idle        = 6'b000000, // extra=00 h2fValid_out=0 busy_out[1:0]=00 DVRstrobe_out=0 
  FIFO_setup  = 6'b000101, // extra=00 h2fValid_out=0 busy_out[1:0]=10 DVRstrobe_out=1 
  FIFO_setup2 = 6'b000100, // extra=00 h2fValid_out=0 busy_out[1:0]=10 DVRstrobe_out=0 
  do_read     = 6'b000010, // extra=00 h2fValid_out=0 busy_out[1:0]=01 DVRstrobe_out=0 
  do_write    = 6'b001100, // extra=00 h2fValid_out=1 busy_out[1:0]=10 DVRstrobe_out=0 
  read_done   = 6'b010010, // extra=01 h2fValid_out=0 busy_out[1:0]=01 DVRstrobe_out=0 
  read_setup  = 6'b000011, // extra=00 h2fValid_out=0 busy_out[1:0]=01 DVRstrobe_out=1 
  start_read  = 6'b100010, // extra=10 h2fValid_out=0 busy_out[1:0]=01 DVRstrobe_out=0 
  start_write = 6'b010100, // extra=01 h2fValid_out=0 busy_out[1:0]=10 DVRstrobe_out=0 
  write_done  = 6'b100100; // extra=10 h2fValid_out=0 busy_out[1:0]=10 DVRstrobe_out=0 

  reg [5:0] state;
  reg [5:0] nextstate;
  reg [7:0] count;
  reg [7:0] next_count;

  // comb always block
  always @* begin
    nextstate = state; // default to hold value because implied_loopback is set
    next_count[7:0] = count[7:0];
    case (state)
      Idle       : begin
        if (ctrl_in[1]) begin
          nextstate = start_write;
          next_count[7:0] = count_in-1;
        end
        else if (ctrl_in[0]) begin
          nextstate = start_read;
          next_count[7:0] = count_in-1;
        end
      end
      FIFO_setup : begin
        begin
          nextstate = FIFO_setup2;
        end
      end
      FIFO_setup2: begin
        begin
          nextstate = do_write;
        end
      end
      do_read    : begin
        if (count == 8'd0) begin
          nextstate = read_done;
        end
        else if (f2hValid_in) begin
          nextstate = do_read;
          next_count[7:0] = count-1;
        end
      end
      do_write   : begin
        if (count == 8'd0) begin
          nextstate = write_done;
        end
        else if (h2fReady_in) begin
          nextstate = do_write;
          next_count[7:0] = count-1;
        end
      end
      read_done  : begin
        begin
          nextstate = Idle;
        end
      end
      read_setup : begin
        begin
          nextstate = do_read;
        end
      end
      start_read : begin
        begin
          nextstate = read_setup;
        end
      end
      start_write: begin
        begin
          nextstate = FIFO_setup;
        end
      end
      write_done : begin
        if (ctrl_in[0]) begin
          nextstate = start_read;
          next_count[7:0] = count_in-1;
        end
        else begin
          nextstate = Idle;
        end
      end
    endcase
  end

  // Assign reg'd outputs to state bits
  assign DVRstrobe_out = state[0];
  assign busy_out[1:0] = state[2:1];
  assign h2fValid_out = state[3];

  // sequential always block
  always @(posedge clk_in) begin
    if (reset_in) begin
      state <= Idle;
      count[7:0] <= 8'd0;
      end
    else begin
      state <= nextstate;
      count[7:0] <= next_count[7:0];
      end
  end

  // datapath sequential always block
  always @(posedge clk_in) begin
    if (reset_in) begin
      RAM_setup_out <= 1'b0;
      chanAddr_out[6:0] <= 7'd0;
      ctrl_out[1:0] <= 2'b00;
      f2hReady_out <= 1'b0;
    end
    else begin
      RAM_setup_out <= 1'b0; // default
      chanAddr_out[6:0] <= chanAddr_out; // default
      ctrl_out[1:0] <= 2'b00; // default
      f2hReady_out <= 1'b0; // default
      case (nextstate)
        FIFO_setup : begin
          RAM_setup_out <= 1'b1;
          ctrl_out[1:0] <= {1'b0,ctrl_in[0]};
        end
        FIFO_setup2: begin
          ctrl_out[1:0] <= {1'b0,ctrl_in[0]};
        end
        do_read    : begin
          f2hReady_out <= 1'b1;
        end
        read_setup : begin
          ctrl_out[1:0] <= {ctrl_in[1], 1'b0};
        end
        start_read : begin
          RAM_setup_out <= 1'b1;
          chanAddr_out[6:0] <= RAMdout_in;
          ctrl_out[1:0] <= {ctrl_in[1], 1'b0};
        end
        start_write: begin
          RAM_setup_out <= 1'b1;
          chanAddr_out[6:0] <= RAMdout_in;
          ctrl_out[1:0] <= {1'b0,ctrl_in[0]};
        end
      endcase
    end
  end

  // This code allows you to see state names in simulation
  `ifndef SYNTHESIS
  reg [87:0] statename;
  always @* begin
    case (state)
      Idle       :
        statename = "Idle";
      FIFO_setup :
        statename = "FIFO_setup";
      FIFO_setup2:
        statename = "FIFO_setup2";
      do_read    :
        statename = "do_read";
      do_write   :
        statename = "do_write";
      read_done  :
        statename = "read_done";
      read_setup :
        statename = "read_setup";
      start_read :
        statename = "start_read";
      start_write:
        statename = "start_write";
      write_done :
        statename = "write_done";
      default    :
        statename = "XXXXXXXXXXX";
    endcase
  end
  `endif

endmodule

