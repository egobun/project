module debounce(    input           clk_in, //clock in
                    input           rst_in, //reset in
                    input           bouncey_in,//raw input to the system
                    output logic    clean_out //debounced output
    );
   logic [19:0] count; // is 20 bits enough?
   logic old;
   always_ff @(posedge clk_in) begin
        old <= bouncey_in;
        if(rst_in)begin
            count <= 20'd0;
            clean_out <= 0;
        end
        else if(bouncey_in == old) begin
            count++;
            if(count == 20'd1000000 && bouncey_in == 1)begin
                clean_out <= 1;
                count <= 20'd0;
            end
            else if(count == 20'd1000000 && bouncey_in == 0)begin
                clean_out <= 0;
                count <= 20'd0;
            end
        end
   end

endmodule