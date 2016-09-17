//======================================================================
//
// m6502_alu.v
// -----------
// Implementation of a MOS 6502 compatible ALU. The ALU is purely
// combinational. The ALU generate both operation result and flag
// results for the operation.
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

module m6502_alu(
                 input wire  [7 : 0] operation,
                 input wire  [7 : 0] op_a,
                 input wire  [7 : 0] op_b,
                 input wire          carry_in,
                 output wire [7 : 0] result,
                 output wire         carry,
                 output wire         zero,
                 output wire         overflow
                );

  //----------------------------------------------------------------
  // Defines.
  //----------------------------------------------------------------
  localparam OP_AND = 8'h01;
  localparam OP_OR  = 8'h02;
  localparam OP_XOR = 8'h03;
  localparam OP_NOT = 8'h04;

  localparam OP_ASL = 8'h11;
  localparam OP_ROL = 8'h12;
  localparam OP_ASR = 8'h13;
  localparam OP_ROR = 8'h14;

  localparam OP_ADD = 8'h21;
  localparam OP_INC = 8'h22;
  localparam OP_SUB = 8'h23;
  localparam OP_DEC = 8'h24;

  localparam OP_CMP = 8'h31;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [7 : 0] tmp_result;
  reg         tmp_carry;
  reg         tmp_zero;
  reg         tmp_overflow;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign result   = tmp_result;
  assign carry    = tmp_carry;
  assign zero     = tmp_zero;
  assign overflow = tmp_overflow;


  //----------------------------------------------------------------
  // alu
  //
  // The actual logic to implement the ALU functions.
  //----------------------------------------------------------------
  always @*
    begin : alu
      reg [8 : 0] tmp_add;

      tmp_result   = 8'h0;
      tmp_carry    = 0;
      tmp_zero     = 0;
      tmp_overflow = 0;

      case (operation)
        OP_AND:
          begin
            tmp_result = op_a & op_b;
          end

        OP_OR:
          begin
            tmp_result = op_a | op_b;
          end

        OP_XOR:
          begin
            tmp_result = op_a ^ op_b;
          end

        OP_NOT:
          begin
            tmp_result = ~op_a;
          end

        OP_ASL:
          begin
            tmp_result = {op_a[6 : 0], carry_in};
            tmp_carry  = op_a[7];
          end

        OP_ROL:
          begin
            tmp_result = {op_a[6 : 0], op_a[7]};
          end

        OP_ASR:
          begin
            tmp_result = {carry_in, op_a[7 : 1]};
            tmp_carry  = op_a[0];
          end

        OP_ROR:
          begin
            tmp_result = {op_a[0], op_a[7 : 1]};
          end

        OP_ADD:
          begin
            tmp_add = op_a + op_b + carry_in;
            tmp_result = tmp_add[7 : 0];
            tmp_carry  = tmp_add[8];
          end

        OP_INC:
          begin
            tmp_add = op_a + 1'b1;
            tmp_result = tmp_add[7 : 0];
            tmp_carry  = tmp_add[8];
          end

        OP_SUB:
          begin
            tmp_result = op_a - op_b;
            if (tmp_result == 8'h00)
              tmp_zero = 1;
          end

        OP_DEC:
          begin
            tmp_result = op_a - 1'b1;
          end

        OP_CMP:
          begin
            if (op_a == op_b)
              tmp_zero = 1;
          end

        default:
          begin
          end
      endcase // case (operation)
    end // alu

endmodule // 6502_alu

//======================================================================
// EOF m6502_alu.v
//======================================================================
