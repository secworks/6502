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

module system_m6502(
                    input wire          clk,
                    input wire          reset_n,

                    output wire [7 : 0] leds
                    );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter MEM_BASE_ADDR = 16'h0000;
  parameter MEM_SIZE      = 16'h03ff;
  parameter IO_LED0_ADDR = 16'h1000;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [7 : 0] mem [0 : 1023];
  reg         mem_we;

  reg [7 : 0] led0_reg;
  reg [7 : 0] led0_new;
  reg         led0_we;

  reg         ready_reg;
  reg         valid_reg;


  //----------------------------------------------------------------
  // Wires for cpu connectivity.
  //----------------------------------------------------------------
  wire          cpu_cs;
  wire          cpu_wr;
  wire [15 : 0] cpu_address;
  reg [7 : 0]   cpu_read_data;
  wire [7 : 0]  cpu_write_data;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign leds = led0_reg;


  //----------------------------------------------------------------
  // Instantiations.
  //----------------------------------------------------------------
  m6502 m6502_cpu(
                  .clk(clk),
                  .reset_n(reset_n),
                  .cs(cpu_cs),
                  .wr(cpu_wr),
                  .address(cpu_address),
                  .mem_ready(ready_reg),
                  .data_valid(valid_reg),
                  .read_data(cpu_read_data),
                  .write_data(cpu_write_data)
                  );


  //----------------------------------------------------------------
  // sync mem and I/O access.
  // TODO Add I/O for reading pins.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin : mem_io_update
      integer i;

      if (!reset_n)
        begin
          led0_reg <= 8'h0;

          for (i = 0 ; i <= MEM_SIZE ; i = i + 1)
            mem[i] <= 8'h0;
        end
      else
        begin
          ready_reg <= 1;
          valid_reg <= 0;

          if ((cpu_address <= MEM_BASE_ADDR) && (cpu_address <= MEM_BASE_ADDR + MEM_SIZE))
            begin
              valid_reg <= 1;
              cpu_read_data <= mem[cpu_address[9 : 0]];
              if (cpu_wr)
                mem[cpu_address[9 : 0]] <= cpu_write_data;
            end

          if (cpu_address == IO_LED0_ADDR)
            begin
              valid_reg <= 1;
              cpu_read_data <= led0_reg;
              if (cpu_wr)
                led0_reg <= cpu_write_data;
            end
        end
    end // mem_io_update

endmodule // system_6502

//======================================================================
// EOF system_m6502.v
//======================================================================
