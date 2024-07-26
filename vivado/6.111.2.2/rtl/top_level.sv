module top_level(
                input logic [15:0] SW,
                input logic BTNC,
                input logic BTND,
                input logic CLK100MHZ,
                output logic [15:0] LED,
                output logic UART_RXD_OUT);

    /// Initial

    assign LED = SW;
    //assign JA[2] = 0; //just assign this to be 0


    /// Debounce

    debounce debounce_u(.clk_in(CLK100MHZ), 
                        .rst_in(BTND), 
                        .bouncey_in(BTNC),
                        .clean_out(clean));

    /// Rising Edge

    logic clean;
    logic old_clean;
    logic rising_clean;

    assign rising_clean = clean & !old_clean;
    always@(posedge CLK100MHZ)begin
        if(BTND)begin
            old_clean <= 1'b0;
        end
        else begin
            old_clean <= clean;
        end
    end

    /// Serial_tx

    serial_tx my_tx(.clk_in(CLK100MHZ), 
                .rst_in(BTND), 
                .trigger_in(rising_clean),
                .val_in(SW[7:0]),
                .data_out(UART_RXD_OUT));
    

    
endmodule