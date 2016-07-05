//======================================================================
//
// system_m6502.v
// --------------
// A simple test system that can be implemented a FPGA device.
// It basically instantiates a m6502 cpu core, provides some
// memory and some address mapped I/O.
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

system_m6502(
             input wire            clk,
             input wire            reset_n,

             output wire [7 : 0]   leds
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


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign leds = 8'h55;


  //----------------------------------------------------------------
  // Instantiations.
  //----------------------------------------------------------------
  m6502 m6502_cpu(
                  .clk(clk),
                  .reset_n(reset_n),
                  .cs(),
                  .wr(),
                  .address(),
                  .mem_ready(),
                  .data_valid(),
                  .read_data(),
                  .write_data()
                  );


  //----------------------------------------------------------------
  // reg_update
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin : reg_update
      integer i;

      if (!reset_n)
        begin

        end
      else
        begin

        end
    end // reg_update

endmodule // 6502

//======================================================================
// EOF system_m6502.v
//======================================================================
