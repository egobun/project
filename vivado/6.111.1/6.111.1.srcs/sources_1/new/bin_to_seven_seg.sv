`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09.06.2024 20:11:43
// Design Name: 
// Module Name: bin_to_seven_seg
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


module bin_to_seven_seg(
    input logic [3:0] bin_in,
    output logic [6:0] hex_out
    );
    always@ (bin_in)begin
      if(bin_in == 0)begin
              hex_out[0] = 1;
              hex_out[1] = 1;
              hex_out[2] = 1;
              hex_out[3] = 1;
              hex_out[4] = 1;
              hex_out[5] = 1;
              hex_out[6] = 0;
        end
      else if(bin_in == 4'b0001)begin
              hex_out[0] = 0;
              hex_out[1] = 1;
              hex_out[2] = 1;
              hex_out[3] = 0;
              hex_out[4] = 0;
              hex_out[5] = 0;
              hex_out[6] = 0;
        end
      else if(bin_in == 4'b0010)  begin
              hex_out[0] = 1;
              hex_out[1] = 1;
              hex_out[2] = 0;
              hex_out[3] = 1;
              hex_out[4] = 1;
              hex_out[5] = 0;
              hex_out[6] = 1;
        end
      else if(bin_in == 4'b0011)  begin
              hex_out[0] = 1;
              hex_out[1] = 1;
              hex_out[2] = 1;
              hex_out[3] = 1;
              hex_out[4] = 0;
              hex_out[5] = 0;
              hex_out[6] = 1;
        end
            if(bin_in == 4'd4)begin
              hex_out[0] = 0;
              hex_out[1] = 1;
              hex_out[2] = 1;
              hex_out[3] = 0;
              hex_out[4] = 0;
              hex_out[5] = 1;
              hex_out[6] = 1;
        end
      else if(bin_in == 4'd5)begin
              hex_out[0] = 1;
              hex_out[1] = 0;
              hex_out[2] = 1;
              hex_out[3] = 1;
              hex_out[4] = 0;
              hex_out[5] = 1;
              hex_out[6] = 1;
        end
      else if(bin_in == 4'd6)  begin
              hex_out[0] = 1;
              hex_out[1] = 0;
              hex_out[2] = 1;
              hex_out[3] = 1;
              hex_out[4] = 1;
              hex_out[5] = 1;
              hex_out[6] = 1;
        end
      else if(bin_in == 4'd7)  begin
              hex_out[0] = 1;
              hex_out[1] = 1;
              hex_out[2] = 1;
              hex_out[3] = 0;
              hex_out[4] = 0;
              hex_out[5] = 0;
              hex_out[6] = 0;
        end
      else if(bin_in == 4'd8)  begin
              hex_out[0] = 1;
              hex_out[1] = 1;
              hex_out[2] = 1;
              hex_out[3] = 1;
              hex_out[4] = 1;
              hex_out[5] = 1;
              hex_out[6] = 1;
        end
      else if(bin_in == 4'd9)  begin
              hex_out[0] = 1;
              hex_out[1] = 1;
              hex_out[2] = 1;
              hex_out[3] = 1;
              hex_out[4] = 0;
              hex_out[5] = 1;
              hex_out[6] = 1;
        end
     else if(bin_in == 4'd10)  begin
              hex_out[0] = 1;
              hex_out[1] = 1;
              hex_out[2] = 1;
              hex_out[3] = 0;
              hex_out[4] = 1;
              hex_out[5] = 1;
              hex_out[6] = 1;
        end
            if(bin_in == 4'd11)begin
              hex_out[0] = 0;
              hex_out[1] = 0;
              hex_out[2] = 1;
              hex_out[3] = 1;
              hex_out[4] = 1;
              hex_out[5] = 1;
              hex_out[6] = 1;
        end
      else if(bin_in == 4'd12)begin
              hex_out[0] = 1;
              hex_out[1] = 0;
              hex_out[2] = 0;
              hex_out[3] = 1;
              hex_out[4] = 1;
              hex_out[5] = 1;
              hex_out[6] = 0;
        end
      else if(bin_in == 4'd13)  begin
              hex_out[0] = 0;
              hex_out[1] = 1;
              hex_out[2] = 1;
              hex_out[3] = 1;
              hex_out[4] = 1;
              hex_out[5] = 0;
              hex_out[6] = 1;
        end
      else if(bin_in == 4'd14)  begin
              hex_out[0] = 1;
              hex_out[1] = 0;
              hex_out[2] = 0;
              hex_out[3] = 1;
              hex_out[4] = 1;
              hex_out[5] = 1;
              hex_out[6] = 1;
        end
      else if(bin_in == 4'd15)  begin
              hex_out[0] = 1;
              hex_out[1] = 0;
              hex_out[2] = 0;
              hex_out[3] = 0;
              hex_out[4] = 1;
              hex_out[5] = 1;
              hex_out[6] = 1;
        end
    end


endmodule
