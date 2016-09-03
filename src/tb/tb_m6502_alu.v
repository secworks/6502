// Testbench for m6502 ALU.

module tb_m6502_alu;

  reg [7 : 0]  tb_operation;
  reg [7 : 0]  tb_op_a;
  reg [7 : 0]  tb_op_b;
  reg          tb_carry_in;
  wire [7 : 0] tb_result;
  wire         tb_carry;
  wire         tb_zero;
  wire         tb_overflow;
  reg [31 : 0]  error_ctr;
  reg [31 : 0]  tc_ctr;



  //----------------------------------------------------------------
  // dut
  //----------------------------------------------------------------
  m6502_alu dut(
                .operation(tb_operation),
                .op_a(tb_op_a),
                .op_b(tb_op_b),
                .carry_in(tb_carry_in),
                .result(tb_result),
                .carry(tb_carry),
                .zero(tb_zero),
                .overflow(tb_overflow)
               );


  //----------------------------------------------------------------
  // display_test_results()
  //
  // Display the accumulated test results.
  //----------------------------------------------------------------
  task display_test_results;
    begin
      if (error_ctr == 0)
        begin
          $display("*** All %02d test cases completed successfully", tc_ctr);
        end
      else
        begin
          $display("*** %02d tests completed - %02d test cases did not complete successfully.",
                   tc_ctr, error_ctr);
        end
    end
  endtask // display_test_results


  //----------------------------------------------------------------
  // init_sim()
  //----------------------------------------------------------------
  task init_sim;
    begin
      tc_ctr       = 0;
      error_ctr    = 0;
      tb_operation = 8'h0;
      tb_op_a      = 8'h0;
      tb_op_b      = 8'h0;
      tb_carry_in  = 0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // main
  //----------------------------------------------------------------
  initial
    begin : main
      $display("   -= Testbench for m6502 ALU started =-");
      $display("    ====================================");
      $display("");

      init_sim();
      display_test_results();

      $display("");
      $display("*** m6502 ALU simulation done. ***");
      $finish;
    end // main

endmodule // tb_m6502_alu
