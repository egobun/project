`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.06.2024 15:36:18
// Design Name: 
// Module Name: simple_counter
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


module simple_counter(  input               clk_in,
                        input               rst_in,
                        input               evt_in,
                        output logic[15:0]  count_out
    );
    
    always_ff @(posedge clk_in)begin
        if(rst_in)begin
            count_out <= 16'b0; //reset signal
        end else if(evt_in) begin
            if(count_out == 16'd65535)begin
                count_out <= 16'b0;
            end
            else begin
                count_out++;
            end       
        end
    end
endmodule
