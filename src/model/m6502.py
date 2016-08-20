#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=======================================================================
#
# m6502.py
# --------
# This is a model of a MOS 6502 CPU. The model is used to verify
# the HW design.
#
#
# Author: Joachim Strömbergson
# Copyright (c) 2016 Secworks Sweden AB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#=======================================================================

#-------------------------------------------------------------------
# Python module imports.
#-------------------------------------------------------------------
import sys
import os


#-------------------------------------------------------------------
# CPU()
#-------------------------------------------------------------------
class M6502:
    #---------------------------------------------------------------
    # At init set
    #---------------------------------------------------------------
    def __init__(self, verbose = False):
        self.mem   = [0] * 65536
        self.pc    = 0
        self.acc   = 0
        self.x_reg = 0
        self.y_reg = 0
        self.carry = 0
        self.zero  = 0


    #---------------------------------------------------------------
    # load()
    # Load the contents of the file with the given name into
    # the memory starting at the given address. If no address is
    # given, loading will start at address 0x0000.
    #---------------------------------------------------------------
    def load(filename, address = 0x0000):
        pass
#        with open(filename, 'rb') as hex_file:
#            for line in hex_file:
#                print(line)
#                    if "#" not in line:
#                        self.mem[address] = line[2:4]
#                        address += 1
#        except IOError:
#            print("file %s can not be opened." % filename)


    #---------------------------------------------------------------
    # execute()
    # Execute a given number of instructions starting from the
    # given address. If no address is given, the execution starts
    # from address zero. If number of instructions are not given
    # execution continues until reaching a zero or break.
    #---------------------------------------------------------------
    def execute(self, start_address = 0x0000, num_instructions = 0):
        pass


    #---------------------------------------------------------------
    # dump_state()
    # Dump the state of all interna registers.
    #---------------------------------------------------------------
    def dump_state(self):
        print("CPU state:")
        print("program counter: 0x%04x" % self.pc)
        print("acc: 0x%02x, x_reg: 0x%02x, y_reg: 0x%02x" %
                  (self.acc, self.x_reg, self.y_reg))
        print("")


    #---------------------------------------------------------------
    # dump_mem()
    # Dump the contents of the memory starting from the given
    # address and length number of bytes. If no address is given
    # dump will start from address 0x000. If no length is given
    # dump will continue up to adress 0xffff
    #---------------------------------------------------------------
    def dump_mem(self, start = 0x0000, length = 0):
        print("Dumping memory from 0x%04x to 0x%04x" % (start, start + length))
        for addr in range (start, start + length + 1):
            print("0x%04x: 0x%02x" % (addr, self.mem[addr]))
        print("")


#-------------------------------------------------------------------
# test_cpu()
#
# Loads a known cpu definition and print the resulting cpu.
#-------------------------------------------------------------------
def test_cpu():
    my_cpu = M6502(True)
    my_cpu.load("example1.hex")
    my_cpu.execute(num_instructions = 10)
    my_cpu.dump_state()
    my_cpu.dump_mem(length = 10)


#-------------------------------------------------------------------
# __name__
# Python thingy which allows the file to be run standalone as
# well as parsed from within a Python interpreter.
#-------------------------------------------------------------------
if __name__=="__main__":
    # Run the main function.
    sys.exit(test_cpu())

#=======================================================================
# EOF m6502.py
#=======================================================================
