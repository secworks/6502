//======================================================================
//
// 6502.v
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

module 65026(
             // Clock and reset.
             input wire            clk,
             input wire            reset_n,

             // Data ports.
             output wire           cs,
             output wire           wr,
             output wire  [15 : 0] address,
             input wire   [7 : 0]  read_data,
             output wire  [7 : 0]  write_data
            );

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------


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
  reg          cs_we

  reg          wr_reg;
  reg          wr_new;
  reg          wr_we

  reg [15 : 0] addr_reg;
  reg [15 : 0] addr_new;
  reg          addr_we;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign cs      = cs_reg;
  assign wr      = wr_reg;
  assign address = addr_reg;


  //----------------------------------------------------------------
  // reg_update
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin : reg_update
      integer i;

      if (!reset_n)
        begin
          a_reg    <= 0;
          x_reg    <= 0;
          y_reg    <= 0;
          wr_reg   <= 0;
          cs_reg   <= 0;
          addr_reg <= 16'h0;
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
            cs_res <= cs_new;

          if (wr_we)
            wr_reg <= wr_new;

          if (addr_we)
            addr_reg <= addr_new;

        end
    end // reg_update

endmodule // 6502

//======================================================================
// EOF 6502.v
//======================================================================
