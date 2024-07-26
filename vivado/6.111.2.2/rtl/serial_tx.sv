module serial_tx(
                input [7:0] val_in,
                input trigger_in,
                input rst_in,
                input clk_in,
                output logic data_out = 1'b1);

    parameter   DIVISOR = 868; //treat this like a constant!!
    
    logic [9:0]         shift_buffer = 10'd512; //10 bits...interesting
    logic [31:0]        count;
    logic [3:0]         i;
    logic               btn_was_pressed;
    logic step = 1'b0;
 
    //assign btn_was_pressed = 1'b0;
    //assign i = 3'd0;

    always_ff @(posedge clk_in) begin
        if (rst_in) begin
            btn_was_pressed <= 1'b0;
            count <= 32'd0;
            i <= 4'd0;
            data_out <= 1'b0;
        end else if (trigger_in) begin
            btn_was_pressed <= 1'b1;
            shift_buffer[8:1] <= val_in; // Добавляем стартовый и стоповый биты
        end else if (btn_was_pressed) begin
            if (count == DIVISOR || step == 0) begin
                step = 1'b1;
                count <= 32'd0;
                data_out <= shift_buffer[i];
                if (i == 9) begin
                    i <= 4'd0;
                    btn_was_pressed <= 1'b0;
                end else begin
                    i <= i + 1;
                end
            end else begin
                count <= count + 1;
            end
        end
    end

    // always @(posedge clk_in)begin
    //     if(rst_in)begin
    //         btn_was_pressed <= 1'b0;
    //         count <= 32'd0;
    //         i <= 3'd0;
    //     end
    //     else if(trigger_in == 1'b1)begin
    //         btn_was_pressed <= 1'b1;
    //     end 
    //     else if(btn_was_pressed == 1'b1)begin
    //         if(btn_was_pressed == 1'b1)begin
    //             if(count == DIVISOR)begin
    //                 count <= 32'd0;
    //                 data_out <= val_in[i];
    //                 if(i == 7)begin
    //                     i <= 3'd0;
    //                     btn_was_pressed <= 1'b0;
    //                 end else begin
    //                     i++;
    //                 end
    //             end
    //             else count++;
    //         end
    //     end 
    // end   

    // always_comb
    //     if(btn_was_pressed == 1'b1)begin
    //             if(count == DIVISOR)begin
    //                 count <= 32'd0;
    //                 data_out <= val_in[i];
    //                 if(i == 7)begin
    //                     i <= 3'd0;
    //                     btn_was_pressed <= 1'b0;
    //                 end else begin
    //                     i++;
    //                 end
    //             end
    //             else count++;
    //     end  


    
    // always_ff @(posedge clk_in) begin
    //     if (rst_in) begin
    //         btn_was_pressed <= 1'b0;
    //         count <= 32'd0;
    //         i <= 3'd0;
    //     end else if (trigger_in && !btn_was_pressed) begin
    //         btn_was_pressed <= 1'b1;
    //     end else if (btn_was_pressed) begin
    //         if (count == DIVISOR) begin
    //             count <= 32'd0;
    //             data_out <= val_in[i];
    //             if (i == 7) begin
    //                 i <= 3'd0;
    //                 btn_was_pressed <= 1'b0;
    //             end else begin
    //                 i <= i + 1;
    //             end
    //         end else begin
    //             count <= count + 1;
    //         end
    //     end
    // end
endmodule
