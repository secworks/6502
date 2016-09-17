#===================================================================
#
# Makefile
# --------
# Makefile for building m6502 cpu, sub module as well as system
# simulation.
#
#
# Author: Joachim Strombergson
# Copyright (c) 2016, Secworks Sweden AB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or
# without modification, are permitted provided that the following
# conditions are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#===================================================================

ALU_SRC=../src/rtl/m6502_alu.v
ALU_TB=../src/tb/tb_m6502_alu.v
CPU_SRC=../src/rtl/m6502.v ../src/rtl/m6502_decoder.v  ../src/rtl/m6502_alu.v

SYSTEM_TB=../src/system/tb/tb_system_m6502.v
SYSTEM_SRC=../src/system/system_m6502.v $(CPU_SRC)

CC=iverilog
LINT=verilator


all: system.sim cpu.sim alu.sim


system.sim:  $(SYSTEM_TB) $(SYSTEM_SRC)
	$(CC) -Wall -o system.sim $(SYSTEM_TB) $(SYSTEM_SRC)


cpu.sim:  $(CPU_SRC)
	$(CC) -Wall -o cpu.sim $(CPU_SRC)


alu.sim:   $(ALU_TB) $(ALU_SRC)
	$(CC) -Wall -o alu.sim $(ALU_TB) $(ALU_SRC)


lint:
	verilator -Wall --lint-only  $(CPU_SRC)


clean:
	rm -f alu.sim
	rm -f cpu.sim
	rm -f system.sim


help:
	@echo "Supported targets:"
	@echo "------------------"
	@echo "all:   Build all simulation targets."
	@echo "lint:  Run the linter on both the block based and the stream based top levels."
	@echo "clean: Remove all generated files."

#===================================================================
# EOF Makefile
#===================================================================
