`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.06.2024 16:31:04
// Design Name: 
// Module Name: leds_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module leds_tb;

        logic [3:0] SW;
        logic [6:0] LED;
        

        /*leds uut(.SW(SW),
                .CLK100MHZ(CLK100MHZ),
                .LED(LED),
                .LED16_R(LED16_R),
                .LED16_G(LED16_G),
                .LED16_B(LED16_B),
                .BTNC(BTNC),
                .BTNL(BTNL),
                .BTNR(BTNR),
                .CA(CA),
                .CB(CB),
                .CC(CC),
                .CD(CD),
                .CE(CE),
                .CF(CF),
                .CG(CG),
                .DP(DP),
                .AN(AN)
                );
        */
 
        bin_to_seven_seg my_display(SW, LED);
    
        initial begin
                $display("Starting Sim");
                SW = 4'b0001; //first input...set it to be "1"
                #10;//let the sim run a little bit (SystemVerilog needs some time to take place!)
                $display("Input is: %4b, Output is %7b",SW,LED);
                //Now check that a binary 1 input results in a hex-display "1":
                assert(LED==7'b0000110) else $error("Didn't draw 1 right :/");
                //you do the rest!
                $finish;
        end

endmodule
