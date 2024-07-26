module top_level(
    input logic [15:0] SW,
    input logic BTNC,
    input logic BTNL,
    input logic BTNU,
    input logic BTND,
    input logic CLK100MHZ,
    output logic CA,
    output logic CB,
    output logic CC,
    output logic CD,
    output logic CE,
    output logic CF,
    output logic CG,
    output logic [7:0] AN
    );

    /// Global parametrs

    logic clean;
    logic [15:0] count_out_btn;
    logic [15:0] count_out_tim;
    logic reset;
    logic [6:0] seg;
    logic [31:0] result; 

    /// Initial

    assign reset = BTND; 
    assign result[31:16] = 32'd0;

    /// Debounce

    debounce debounce_u(.clk_in(CLK100MHZ), 
                        .rst_in(BTND), 
                        .bouncey_in(BTNU),
                        .clean_out(clean));

    /// Rising Edge

    logic old_clean;
    logic rising_clean;

    assign rising_clean = clean & !old_clean;
    always@(posedge CLK100MHZ)begin
        if(reset)begin
            old_clean <= 1'b0;
        end
        else begin
            old_clean <= clean;
        end
    end

    /// 10 Hz pulse

    parameter PERIOD = 32'd10_000_000;
    logic[31:0] periodic_counter; //32 bits is more than sufficient!
    logic pulse;
    assign pulse = (periodic_counter == PERIOD-1);

    always_ff@ (posedge CLK100MHZ) begin
        if(reset)begin
            periodic_counter <= 32'd0;
        end
        else begin
            if(periodic_counter == PERIOD - 1)begin
                periodic_counter <= 32'd0;
            end
            else begin
                periodic_counter <= periodic_counter + 32'b1;
            end
        end
    end

    /// simple_counter btn

    simple_counter simple_counter_btn(.clk_in(CLK100MHZ),
                                      .rst_in(BTND),
                                      .evt_in(rising_clean),
                                      .count_out(count_out_btn));

    /// simple_counter timer(дописать)

    simple_counter simple_counter_tim(.clk_in(CLK100MHZ),
                                      .rst_in(BTND),
                                      .evt_in(pulse),
                                      .count_out(count_out_tim));                                         

    /// Final_step--------seven_seg_controller

    
    //logic [31:0] switch;
    
    assign seg[6:0] = {CG,CF,CE,CD,CC,CB,CA}; 
    //assign result[31:16] = 32'd0;
    //assign result[15:0] = SW[15:0];
    seven_seg_controller seven_seg (.clk_in(CLK100MHZ),
                                    .rst_in(BTND),
                                    .val_in(result),
                                    .cat_out(seg),
                                    .an_out(AN));

    /// Mux

    always_comb
        if(BTNL == 1)begin
            result[15:0] = count_out_btn;
        end
        else if(BTNL == 0)begin
            if(BTNC == 1)begin
                result[15:0] = count_out_tim;
            end
            else begin
                result[15:0] = SW[15:0];
            end
        end
    
endmodule