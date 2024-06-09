module leds
    (
        input logic [3:0] SW,
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

    logic [6:0] log_out;
    assign AN[7] = 1'b0;
    assign AN[6] = 1'b0;
    assign AN[5] = 1'b0;
    assign AN[4] = 1'b0;
    assign AN[3] = 1'b0;
    assign AN[2] = 1'b0;
    assign AN[1] = 1'b0;
    assign AN[0] = 1'b0;
    bin_to_seven_seg my_convert(SW,log_out);
    assign {CG,CF,CE,CD,CC,CB,CA} = ~log_out;
    /*
    always@ (posedge CLK100MHZ)begin
        if(SW[1] == 1'b1)begin
            CA <= 1;
            CB <= 0;
            CC <= 0;
            CD <= 1;
            CE <= 1;
            CF <= 1;
            CG <= 1;
        end
        else if(SW[2] == 1'b1)begin
            CA <= 0;
            CB <= 0;
            CC <= 1;
            CD <= 0;
            CE <= 0;
            CF <= 1;
            CG <= 0;
        end
        else if(SW[3] == 1'b1)begin
            CA <= 0;
            CB <= 0;
            CC <= 0;
            CD <= 0;
            CE <= 1;
            CF <= 1;
            CG <= 0;
        end
    end
    */
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