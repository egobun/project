`timescale 1ns / 1ps

module top_level_tb;

    logic [3:0] sw;
    logic [6:0] led_out;    

    bin_to_seven_seg my_display(sw, led_out);
    
    initial begin
            $display("Starting Sim");
            sw = 4'b0001; //first input...set it to be "1"
            #10;//let the sim run a little bit (SystemVerilog needs some time to take place!)
            $display("Input is: %4b, Output is %7b",sw,led_out);
            //Now check that a binary 1 input results in a hex-display "1":
            assert(led_out==7'b0000110) else $error("Didn't draw 1 right :/");
            //you do the rest!
            $finish;
    end

endmodule