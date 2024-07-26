`timescale 1ns / 1ps

module simple_counter_tb;
    
    //make logics for inputs and outputs!
    logic clk;
    logic rst;
    logic evt;
    logic[15:0] count;  //size appropriately!

    //make an instance of the counter...call it uut or my_counter or something else
    simple_counter my_counter(  .clk_in(clk),
                                .rst_in(rst),
                                .evt_in(evt),
                                .count_out(count));                            
    
    //An always block in simulation **always** runs in the background
    //this is your standard way of making a clock below:
    //it says: every 5 ns, make clk be !clk
    //still need to initialize clk in an initial block
    always begin
        #5;  //every 5 ns switch...so period of clock is 10 ns...100 MHz clock
        clk = !clk;
    end
    
    //initial block...this is our test simulation
    initial begin
        $display("Starting Sim"); //print nice message
        clk = 0; //initialize clk (super important)
        rst = 0; //initialize rst (super important)
        evt = 0; //initialize evt (super important)
        #20  //wait a little bit of time at beginning
        rst = 1; //reset system
        #20; //hold high for a few clock cycles
        rst=0; //pull low
        #20; //wait a little bit
        evt = 1; //make an evt
        #10 //wait a clock cycle
        evt = 0; //pull low
        #20 //wait 50 ns
        evt = 1; //pull high
        #20 //just let it run for a bit
        evt = 0; //pull low, and be done
        $finish;
    
  
    end
endmodule //counter_tb