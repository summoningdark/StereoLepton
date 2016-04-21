
// Created by fizzim.pl version $Revision: 5.0 on 2015:11:08 at 20:24:02 (www.fizzim.com)

module commFPGA_I2Cmaster_fsm (
  output reg AL_out,
  output reg NAK_out,
  output reg ack_out,
  output reg core_en_out,
  output reg [7:0] fifo_din_out,
  output reg fifo_rd_out,
  output reg fifo_wr_out,
  output reg idle_out,
  output reg read_out,
  output reg reset_out,
  output reg start_out,
  output reg stop_out,
  output reg [7:0] txr_out,
  output reg write_out,
  input wire ack_in,
  input wire [15:0] bytes2read_in,
  input wire clk_in,
  input wire done_in,
  input wire [7:0] fifo_dout_in,
  input wire fifo_empty_in,
  input wire fifo_full_in,
  input wire i2c_al_in,
  input wire i2c_busy_in,
  input wire reset_in,
  input wire [7:0] rxr_in,
  input wire [9:0] slave_addr_in,
  input wire tenbit_in 
);

  // state bits
  parameter 
  reset          = 5'b00000, 
  address        = 5'b00001, 
  address_10a    = 5'b00010, 
  address_10b    = 5'b00011, 
  adr_ack        = 5'b00100, 
  adr_wr         = 5'b00101, 
  bus_busy       = 5'b00110, 
  idle           = 5'b00111, 
  read           = 5'b01000, 
  read_push      = 5'b01001, 
  rs_addr        = 5'b01010, 
  rs_addr10a     = 5'b01011, 
  rs_addr10a_ack = 5'b01100, 
  rs_addr10b     = 5'b01101, 
  rs_addr10b_ack = 5'b01110, 
  rs_addr_ack    = 5'b01111, 
  wait_fifo      = 5'b10000, 
  write          = 5'b10001, 
  write_ack      = 5'b10010, 
  write_fetch    = 5'b10011; 

  reg [4:0] state;
  reg [4:0] nextstate;
  reg [15:0] byte_count;
  reg do_read;
  reg next_AL_out;
  reg next_NAK_out;
  reg [15:0] next_byte_count;

  // comb always block
  always @* begin
    nextstate = state; // default to hold value because implied_loopback is set
    next_AL_out = AL_out;
    next_NAK_out = NAK_out;
    next_byte_count[15:0] = byte_count[15:0];
    case (state)
      reset         : begin
        begin
          nextstate = idle;
        end
      end
      address       : begin
        begin
          nextstate = adr_wr;
        end
      end
      address_10a   : begin
        begin
          nextstate = adr_wr;
        end
      end
      address_10b   : begin
        begin
          nextstate = write;
        end
      end
      adr_ack       : begin
        if (ack_in) begin
          nextstate = reset;
          next_NAK_out = 1'b1;
        end
        else if (tenbit_in) begin
          nextstate = address_10b;
        end
        else begin
          nextstate = write_fetch;
        end
      end
      adr_wr        : begin
        if (i2c_al_in) begin
          nextstate = reset;
          next_AL_out = 1'b1;
        end
        else if (done_in) begin
          nextstate = adr_ack;
        end
      end
      bus_busy      : begin
        if (!i2c_busy_in) begin
          nextstate = idle;
        end
      end
      idle          : begin
        if (!fifo_empty_in & !i2c_busy_in & tenbit_in) begin
          nextstate = address_10a;
          next_AL_out = 1'b0;
          next_byte_count[15:0] = bytes2read_in;
          next_NAK_out = 1'b0;
        end
        else if (!fifo_empty_in & !i2c_busy_in) begin
          nextstate = address;
          next_AL_out = 1'b0;
          next_NAK_out = 1'b0;
          next_byte_count[15:0] = bytes2read_in;
        end
        else if (i2c_busy_in) begin
          nextstate = bus_busy;
        end
      end
      read          : begin
        if (done_in) begin
          nextstate = read_push;
          next_byte_count[15:0] = byte_count - 1;
        end
      end
      read_push     : begin
        if (byte_count == 0) begin
          nextstate = idle;
        end
        else begin
          nextstate = read;
        end
      end
      rs_addr       : begin
        if (i2c_al_in) begin
          nextstate = reset;
          next_AL_out = 1'b1;
        end
        else if (done_in) begin
          nextstate = rs_addr_ack;
        end
      end
      rs_addr10a    : begin
        if (i2c_al_in) begin
          nextstate = reset;
          next_AL_out = 1'b1;
        end
        else if (done_in) begin
          nextstate = rs_addr10a_ack;
        end
      end
      rs_addr10a_ack: begin
        if (ack_in) begin
          nextstate = reset;
          next_NAK_out = 1'b1;
        end
        else begin
          nextstate = rs_addr10b;
        end
      end
      rs_addr10b    : begin
        if (i2c_al_in) begin
          nextstate = reset;
          next_AL_out = 1'b1;
        end
        else if (done_in) begin
          nextstate = rs_addr10b_ack;
        end
      end
      rs_addr10b_ack: begin
        if (ack_in) begin
          nextstate = reset;
          next_NAK_out = 1'b1;
        end
        else begin
          nextstate = read;
        end
      end
      rs_addr_ack   : begin
        if (ack_in) begin
          nextstate = reset;
          next_NAK_out = 1'b1;
        end
        else begin
          nextstate = read;
        end
      end
      wait_fifo     : begin
        begin
          nextstate = write;
        end
      end
      write         : begin
        if (i2c_al_in) begin
          nextstate = reset;
          next_AL_out = 1'b1;
        end
        else if (done_in) begin
          nextstate = write_ack;
        end
      end
      write_ack     : begin
        if (ack_in) begin
          nextstate = reset;
          next_NAK_out = 1'b1;
        end
        else if (!fifo_empty_in) begin
          nextstate = write_fetch;
        end
        else if (do_read & tenbit_in) begin
          nextstate = rs_addr10a;
        end
        else if (do_read) begin
          nextstate = rs_addr;
        end
        else begin
          nextstate = idle;
        end
      end
      write_fetch   : begin
        begin
          nextstate = wait_fifo;
        end
      end
    endcase
  end

  // Assign reg'd outputs to state bits

  // sequential always block
  always @(posedge clk_in) begin
    if (reset_in) begin
      state <= reset;
      AL_out <= 1'b0;
      NAK_out <= 1'b0;
      byte_count[15:0] <= 16'h0000;
      end
    else begin
      state <= nextstate;
      AL_out <= next_AL_out;
      NAK_out <= next_NAK_out;
      byte_count[15:0] <= next_byte_count[15:0];
      end
  end

  // datapath sequential always block
  always @(posedge clk_in) begin
    if (reset_in) begin
      ack_out <= 1'b0;
      core_en_out <= 1'b0;
      do_read <= 1'b0;
      fifo_din_out[7:0] <= 8'h00;
      fifo_rd_out <= 1'b0;
      fifo_wr_out <= 1'b0;
      idle_out <= 1'b0;
      read_out <= 1'b0;
      reset_out <= 1'b1;
      start_out <= 1'b0;
      stop_out <= 1'b0;
      txr_out[7:0] <= 8'h00;
      write_out <= 1'b0;
    end
    else begin
      ack_out <= 1'b0; // default
      core_en_out <= 1'b1; // default
      do_read <= do_read; // default
      fifo_din_out[7:0] <= 8'h00; // default
      fifo_rd_out <= 1'b0; // default
      fifo_wr_out <= 1'b0; // default
      idle_out <= 1'b0; // default
      read_out <= 1'b0; // default
      reset_out <= 1'b0; // default
      start_out <= 1'b0; // default
      stop_out <= 1'b0; // default
      txr_out[7:0] <= txr_out; // default
      write_out <= 1'b0; // default
      case (nextstate)
        reset         : begin
          core_en_out <= 1'b0;
          reset_out <= 1'b1;
        end
        address       : begin
          txr_out[7:0] <= {slave_addr_in[6:0], 1'b0};
        end
        address_10a   : begin
          txr_out[7:0] <= {5'b11110, slave_addr_in[9:8], 1'b0};
        end
        address_10b   : begin
          txr_out[7:0] <= slave_addr_in[7:0];
        end
        adr_wr        : begin
          start_out <= 1'b1;
          write_out <= 1'b1;
        end
        idle          : begin
          do_read <= bytes2read_in != 0;
          idle_out <= 1'b1;
        end
        read          : begin
          ack_out <= byte_count == 1;
          read_out <= 1'b1;
          stop_out <= byte_count == 1;
        end
        read_push     : begin
          fifo_din_out[7:0] <= rxr_in;
          fifo_wr_out <= 1'b1;
        end
        rs_addr       : begin
          start_out <= 1'b1;
          txr_out[7:0] <= {slave_addr_in[6:0], 1'b1};
          write_out <= 1'b1;
        end
        rs_addr10a    : begin
          start_out <= 1'b1;
          txr_out[7:0] <= {5'b11110, slave_addr_in[9:8], 1'b1};
          write_out <= 1'b1;
        end
        rs_addr10b    : begin
          txr_out[7:0] <= slave_addr_in[7:0];
          write_out <= 1'b1;
        end
        write         : begin
          stop_out <= fifo_empty_in & !do_read;
          write_out <= 1'b1;
        end
        write_fetch   : begin
          fifo_rd_out <= 1'b1;
          txr_out[7:0] <= fifo_dout_in;
        end
      endcase
    end
  end

  // This code allows you to see state names in simulation
  `ifndef SYNTHESIS
  reg [111:0] statename;
  always @* begin
    case (state)
      reset         :
        statename = "reset";
      address       :
        statename = "address";
      address_10a   :
        statename = "address_10a";
      address_10b   :
        statename = "address_10b";
      adr_ack       :
        statename = "adr_ack";
      adr_wr        :
        statename = "adr_wr";
      bus_busy      :
        statename = "bus_busy";
      idle          :
        statename = "idle";
      read          :
        statename = "read";
      read_push     :
        statename = "read_push";
      rs_addr       :
        statename = "rs_addr";
      rs_addr10a    :
        statename = "rs_addr10a";
      rs_addr10a_ack:
        statename = "rs_addr10a_ack";
      rs_addr10b    :
        statename = "rs_addr10b";
      rs_addr10b_ack:
        statename = "rs_addr10b_ack";
      rs_addr_ack   :
        statename = "rs_addr_ack";
      wait_fifo     :
        statename = "wait_fifo";
      write         :
        statename = "write";
      write_ack     :
        statename = "write_ack";
      write_fetch   :
        statename = "write_fetch";
      default       :
        statename = "XXXXXXXXXXXXXX";
    endcase
  end
  `endif

endmodule

