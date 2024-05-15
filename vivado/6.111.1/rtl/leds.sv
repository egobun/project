module leds
    (
        input wire [2:0] SW,
        output logic[3:0] LED,
        output logic LED16_R,
        output logic LED16_G,
        output logic LED16_B,
        input BTNC,
        input BTNL,
        input BTNR,
        output logic CA,
        output logic CB,
        output logic CC,
        output logic CD,
        output logic CE,
        output logic CF,
        output logic CG,
        output logic DP,
        output logic [7:0] AN
    );
    assign LED16_R = BTNL;
    assign LED16_G = BTNC;
    assign LED16_B = BTNR;
    
    assign AN[7] = SW[0];
    always @ (SW[0])
    begin
    if(!SW[0])
        begin
            assign CC = 0;
            assign CB = 0;
        end
    end
    assign CA = SW[1];
    
    /* 
    always@ (SW[0])
    while(SW[0] != 0)
        begin
            assign LED16_R = 1'b1;
            assign LED16_G = 1'b0;
            assign LED16_B = 1'b0;
        end
    
    always@ (SW[1])
    while(SW[1] != 0)
        begin
            assign LED16_R = 1'b0;
            assign LED16_G = 1'b1;
            assign LED16_B = 1'b0;
        end
     
    always@ (SW[2])
    while(SW[2] != 0)
        begin
            assign LED16_R = 1'b0;
            assign LED16_G = 1'b0;
            assign LED16_B = 1'b1;
        end
   */
    /*assign LED[0] = SW[0];
    assign LED[1] = SW[1];
    assign LED[2] = SW[0];
    assign LED[3] = SW[1];*/
    
    

endmodule