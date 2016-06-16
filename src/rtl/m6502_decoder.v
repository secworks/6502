//======================================================================
//
// m6502_decoder.v
// ---------------
// Instruction decode logic for the MOS 6502 compatible CPU core.
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

module m6502_decoder(
                     input wire          clk,
                     input wire          reset_n,

                     input wire [7 : 0]  opcode,

                     output wire [1 : 0] instr_len,
                     output wire [1 : 0] opa,
                     output wire [1 : 0] opb,
                     output wire [1 : 0] alu_op,
                     output wire [1 : 0] destination
                    );

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  // Symbolic names for destination codes.
  localparam DEST_MEM  = 2'h0;
  localparam DEST_AREG = 2'h1;
  localparam DEST_XREG = 2'h2;
  localparam DEST_YREG = 2'h3;

  // Symbolic names for ALU operations.
  localparam ALU_NONE = 2'h0;
  localparam ALU_ADC  = 2'h1;

  // M6502 Opcodes:
  localparam OP_BRK     = 8'h00;
  localparam OP_JMP     = 8'h4c;
  localparam OP_TXA     = 8'h8a;
  localparam OP_LDA_IMM = 8'ha9;
  localparam OP_TAX     = 8'haa;
  localparam OP_DEX     = 8'hca;
  localparam OP_INX     = 8'he8;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [1 : 0] ilen;
  reg [1 : 0] dest;
  reg [1 : 0] alu;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign instr_len   = ilen;
  assign destination = dest;
  assign alu_op      = alu;


  //----------------------------------------------------------------
  // decoder
  // Instruction decoder. Based on the opcode provides info about
  // the operation to perform
  //----------------------------------------------------------------
  always @*
    begin : decoder
      ilen = 2'h0;
      dest = DEST_MEM;
      alu  = ALU_NONE;

      case (opcode)
        OP_LDA_IMM:
          begin
            ilen = 2'h2;
            dest = DEST_AREG;
            alu  = ALU_NONE;
          end

        default:
          begin
          end
      endcase // case (opcode)
    end // decoder

endmodule // 6502_decoder

//======================================================================
// EOF m6502_decoder.v
//======================================================================
