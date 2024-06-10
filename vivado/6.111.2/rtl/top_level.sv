module top_level(
    input logic [15:0] SW,
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

    logic [6:0] seg;
    logic [31:0] switch;
    
    assign seg[6:0] = {CG,CF,CE,CD,CC,CB,CA}; 
    assign switch[31:16] = 32'd0;
    assign switch[15:0] = SW[15:0];


    seven_seg_controller seven_seg (.clk_in(CLK100MHZ),
                                    .rst_in(BTND),
                                    .val_in(switch),
                                    .cat_out(seg),
                                    .an_out(AN));

endmodule