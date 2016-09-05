//======================================================================
//
// m6502.v
// ------
// Implementation of a MOS 6502 compatible CPU core.
// Note: This is not going to be a cycle accurate model.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2016, Secworks Sweden AB
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module m6502(
             input wire            clk,
             input wire            reset_n,

             output wire           cs,
             output wire           wr,
             output wire  [15 : 0] address,
             input wire            mem_ready,
             input wire            data_valid,
             input wire   [7 : 0]  read_data,
             output wire  [7 : 0]  write_data
            );

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam BOOT_ADDR = 16'h0000;

  localparam AMUX_TMP = 1'h0;
  localparam AMUX_PC  = 1'h1;

  localparam DMUX_AREG = 2'h0;
  localparam DMUX_XREG = 2'h1;
  localparam DMUX_YREG = 2'h2;

  localparam M6502_CTRL_IDLE          = 3'h0;
  localparam M6502_CTRL_GET_OPCODE    = 3'h1;
  localparam M6502_CTRL_STORE_OPCODE  = 3'h2;
  localparam M6502_CTRL_DECODE_OPCODE = 3'h3;
  localparam M6502_CTRL_DATA          = 3'h4;
  localparam M6502_CTRL_EXECUTE       = 3'h5;
  localparam M6502_CTRL_STORE_DATA    = 3'h6;
  localparam M6502_CTRL_UPDATE_PC     = 3'h7;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [7 : 0]  a_reg;
  reg [7 : 0]  a_new;
  reg          a_we;

  reg [7 : 0]  x_reg;
  reg [7 : 0]  x_new;
  reg          x_we;

  reg [7 : 0]  y_reg;
  reg [7 : 0]  y_new;
  reg          y_we;

  reg          carry_reg;
  wire         carry_new;
  reg          carry_we;

  reg          zero_reg;
  wire         zero_new;
  reg          zero_we;

  reg          overflow_reg;
  wire         overflow_new;
  reg          overflow_we;

  reg          cs_reg;
  reg          cs_new;
  reg          cs_we;

  reg          wr_reg;
  reg          wr_new;
  reg          wr_we;

  reg [7 : 0]  opcode_reg;
  reg          opcode_we;

  reg [15 : 0] pc_reg;
  reg [15 : 0] pc_new;
  reg          pc_inc;
  reg          pc_set;
  reg          pc_rst;
  reg          pc_we;

  reg [7 : 0]  write_data_reg;
  reg [7 : 0]  write_data_new;
  reg          write_data_we;

  reg [7 : 0]  read_data_reg;
  reg          read_data_we;

  reg [7 : 0]  addr_lo_reg;
  reg [7 : 0]  addr_lo_new;
  reg          addr_lo_we;
  reg [7 : 0]  addr_hi_reg;
  reg [7 : 0]  addr_hi_new;
  reg          addr_hi_we;

  reg [2 : 0]  m6502_ctrl_reg;
  reg [2 : 0]  m6502_ctrl_new;
  reg          m6502_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [15 : 0]  muxed_address;
  wire [1 : 0]  opcode_length;
  wire [15 : 0] data16;

  reg [7 : 0]   muxed_wr_data;

  reg           amux_ctrl;
  reg  [1 : 0]  dmux_ctrl;

  wire [2 : 0]  decoder_ilen;
  wire [2 : 0]  decoder_opa;
  wire [2 : 0]  decoder_opb;
  wire [2 : 0]  decoder_alu_op;
  wire [2 : 0]  decoder_dest;
  wire          decoder_carry;
  wire          decoder_zero;
  wire          decoder_overflow;

  reg [7 : 0]   alu_operation;
  reg [7 : 0]   alu_op_a;
  reg [7 : 0]   alu_op_b;
  wire [7 : 0]  alu_result;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign cs         = cs_reg;
  assign wr         = wr_reg;
  assign address    = muxed_address;
  assign write_data = muxed_wr_data;


  //----------------------------------------------------------------
  // Instantiations.
  //----------------------------------------------------------------
  m6502_decoder decoder(
                        .opcode(opcode_reg),
                        .instr_len(decoder_ilen),
                        .opa(decoder_opa),
                        .opb(decoder_opb),
                        .alu_op(decoder_alu_op),
                        .destination(decoder_dest),
                        .update_carry(decoder_carry),
                        .update_zero(decoder_zero),
                        .update_overflow(decoder_overflow)
                       );

  m6502_alu alu(
                .operation(alu_operation),
                .op_a(alu_op_a),
                .op_b(alu_op_a),
                .carry_in(carry_reg),
                .result(alu_result),
                .carry(carry_new),
                .zero(zero_new),
                .overflow(overflow_new)
               );


  //----------------------------------------------------------------
  // reg_update
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin : reg_update
      integer i;

      if (!reset_n)
        begin
          a_reg          <= 8'h0;
          x_reg          <= 8'h0;
          y_reg          <= 8'h0;
          carry_reg      <= 0;
          zero_reg       <= 0;
          overflow_reg   <= 0;
          wr_reg         <= 0;
          cs_reg         <= 0;
          opcode_reg     <= 8'h0;
          read_data_reg  <= 8'h0;
          write_data_reg <= 8'h0;
          addr_lo_reg    <= 8'h0;
          addr_hi_reg    <= 8'h0;
          pc_reg         <= BOOT_ADDR;
          m6502_ctrl_reg <= M6502_CTRL_IDLE;
        end
      else
        begin
          if (a_we)
            a_reg <= a_new;

          if (x_we)
            x_reg <= x_new;

          if (y_we)
            y_reg <= y_new;

          if (carry_we)
            carry_reg <= carry_new;

          if (zero_we)
            zero_reg <= zero_new;

          if (overflow_we)
            overflow_reg <= overflow_new;

          if (cs_we)
            cs_reg <= cs_new;

          if (wr_we)
            wr_reg <= wr_new;

          if (opcode_we)
            opcode_reg <= read_data;

          if (read_data_we)
            read_data_reg <= read_data;

          if (write_data_we)
            write_data_reg <= write_data_new;

          if (addr_lo_we)
            addr_lo_reg <= addr_lo_new;

          if (addr_hi_we)
            addr_hi_reg <= addr_hi_new;

          if (pc_we)
            pc_reg <= pc_new;

          if (m6502_ctrl_we)
            m6502_ctrl_reg <= m6502_ctrl_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // pc_update
  //
  // Logic for updating the program counter. Right now it supports
  // resetting to the boot vector, increasing by one and setting
  // to whats in the 16 bit data register. There will quite
  // probably be functions for setting based on the stack, based
  // on result from the ALU. That will be handled as suboperations
  // for the pc_set.
  //----------------------------------------------------------------
  always @*
    begin : pc_update
      pc_new = 16'h0;
      pc_we  = 0;

      if (pc_rst)
        begin
          pc_new = BOOT_ADDR;
          pc_we  = 1;
        end

      if (pc_inc)
        begin
          pc_new = pc_reg + 1'h1;
          pc_we = 1;
        end

      if (pc_set)
        begin
          pc_new = data16;
          pc_we = 1;
        end
    end // pc_update


  //----------------------------------------------------------------
  // addr_mux
  //----------------------------------------------------------------
  always @*
    begin : addr_mux
      muxed_address = 16'h0;

      if (amux_ctrl == AMUX_PC)
        muxed_address = pc_reg;
      else
        muxed_address = 16'h0;
    end // addr_mux


  //----------------------------------------------------------------
  // data_mux
  //
  // Mux for selecting source for write data.
  //----------------------------------------------------------------
  always @*
    begin : data_mux
      muxed_wr_data = 16'h0;

      case (dmux_ctrl)
        DMUX_AREG: muxed_wr_data = a_reg;
        DMUX_XREG: muxed_wr_data = x_reg;
        DMUX_YREG: muxed_wr_data = y_reg;
      endcase // case (dmux_ctrl)
    end // data_mux


  //----------------------------------------------------------------
  // m6502_ctrl
  // Main control FSM for the m6502 model.
  //----------------------------------------------------------------
  always @*
    begin : m6502_ctrl
      opcode_we      = 0;
      pc_inc         = 0;
      pc_set         = 0;
      pc_rst         = 0;
      cs_new         = 0;
      cs_we          = 0;
      wr_new         = 0;
      wr_we          = 0;
      amux_ctrl      = AMUX_PC;
      dmux_ctrl      = DMUX_AREG;

      m6502_ctrl_new = M6502_CTRL_IDLE;
      m6502_ctrl_we  = 1;

      case (m6502_ctrl_reg)
        // Initial state after reset. Set PC and go fetch
        // the first instruction.
        M6502_CTRL_IDLE:
          begin
            pc_rst         = 1;
            m6502_ctrl_new = M6502_CTRL_GET_OPCODE;
            m6502_ctrl_we  = 1;
          end

        // Signal the mem system that we want to read a byte.
        M6502_CTRL_GET_OPCODE:
          begin
            cs_new         = 1;
            cs_we          = 1;
            amux_ctrl      = AMUX_PC;
            m6502_ctrl_new = M6502_CTRL_STORE_OPCODE;
            m6502_ctrl_we  = 1;
          end

        // When we get a byte we store it and stop reading from the memory.
        // We also increase the pc so that it is ready for the next read op.
        M6502_CTRL_STORE_OPCODE:
          begin
            if (mem_ready)
              begin
                pc_inc         = 1;
                cs_new         = 0;
                cs_we          = 1;
                opcode_we      = 1;
                m6502_ctrl_new = M6502_CTRL_DECODE_OPCODE;
                m6502_ctrl_we  = 1;
              end
          end

        // The opcode has been decoded and we need to either read more data
        // or directly execute the instruction.
        M6502_CTRL_DECODE_OPCODE:
          begin
          end

        default:
          begin
          end
      endcase // case (m6502_ctrl_reg)
    end // m6502_ctrl

endmodule // 6502

//======================================================================
// EOF m6502.v
//======================================================================
