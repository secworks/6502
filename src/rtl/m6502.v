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
             input wire   [7 : 0]  read_data,
             output wire  [7 : 0]  write_data
            );

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam BOOT_ADDR = 16'h1000;


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

  reg [3 : 0]  m6502_ctrl_reg;
  reg [3 : 0]  m6502_ctrl_new;
  reg          m6502_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [15 : 0]  muxed_address;
  wire [1 : 0]  opcode_length;
  wire [15 : 0] data16;

  wire [2 : 0]  decoder_ilen;
  wire [1 : 0]  decoder_opa;
  wire [1 : 0]  decoder_opb;
  wire [2 : 0]  decoder_alu_op;
  wire [1 : 0]  decoder_dest;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign cs         = cs_reg;
  assign wr         = wr_reg;
  assign address    = muxed_address;
  assign write_data = write_data_reg;


  //----------------------------------------------------------------
  // Instantiations.
  //----------------------------------------------------------------
  m6502_decoder decoder(
                        .instr_len(decoder_ilen),
                        .opa(decoder_opa),
                        .opb(decoder_opb),
                        .alu_op(decoder_alu_op),
                        .destination(decoder_dest)
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
          wr_reg         <= 0;
          cs_reg         <= 0;
          opcode_reg     <= 8'h0;
          read_data_reg  <= 8'h0;
          write_data_reg <= 8'h0;
          addr_lo_reg    <= 8'h0;
          addr_hi_reg    <= 8'h0;
          pc_reg         <= BOOT_ADDR;
        end
      else
        begin
          if (a_we)
            a_reg <= a_new;

          if (x_we)
            x_reg <= x_new;

          if (y_we)
            y_reg <= y_new;

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

        end
    end // reg_update


  //----------------------------------------------------------------
  // alu
  //----------------------------------------------------------------
  always @*
    begin : alu
      reg [7 : 0] opa;
      reg [7 : 0] opb;
      reg [7 : 0] result;

//      case (decoder_opa)
//
//      endcase // case (decoder_opa)
//
//
//
//      case (decoder_opa)
//
//      endcase // case (decoder_opa)
//
//
//
//      case (decoder_alu_op)
//
//      endcase // case (decoder_alu_op)
//
//
//
//      case (decoder_dest)
//
//      endcase // case (decoder_dest)
    end // alu


  //----------------------------------------------------------------
  // pc_update
  // Can inc to next instruction and jump to a given 16 bit address.
  //----------------------------------------------------------------
  always @*
    begin : pc_update
      pc_we = 0;
      pc_new = 16'h0;

      if (pc_inc)
        begin
          pc_we = 1;
          pc_new = pc_reg + opcode_length;
        end

      if (pc_set)
        begin
          pc_we = 1;
          pc_new = data16;
        end
    end // pc_update


  //----------------------------------------------------------------
  // addr_mux
  //----------------------------------------------------------------
  always @*
    begin : addr_mux
      muxed_address = 16'h0;
    end // addr_mux


  //----------------------------------------------------------------
  // m6502_ctrl
  // Main control FSM for the m6502 model.
  //----------------------------------------------------------------
  always @*
    begin : m6502_ctrl
      opcode_we = 0;
      pc_inc    = 0;
      pc_set    = 0;

      case (m6502_ctrl_reg)

        default:
          begin
          end
      endcase // case (m6502_ctrl_reg)
    end // m6502_ctrl

endmodule // 6502

//======================================================================
// EOF m6502.v
//======================================================================
