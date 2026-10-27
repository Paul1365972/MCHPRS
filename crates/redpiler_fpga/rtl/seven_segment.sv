module seven_segment (
    input logic clock,
    input logic [15:0] digits,
    input logic decimal_point,
    // {g, f, e, d, c, b, a}
    output logic [6:0] segment_n,
    output logic [3:0] anode_n,
    output logic decimal_point_n
);
    // At 100 MHz, four digits of 2^16 clocks take 2.6 ms; Digilent's manual asks for 1 to 16 ms.
    localparam int DIGIT_CLOCK_BITS = 16;

    logic [DIGIT_CLOCK_BITS + 1:0] clocks = 0;
    logic [1:0] position;
    logic [6:0] lit;

    assign position = clocks[DIGIT_CLOCK_BITS + 1:DIGIT_CLOCK_BITS];
    assign anode_n = ~(4'b0001 << position);
    assign segment_n = ~lit;
    assign decimal_point_n = !decimal_point;

    always_comb begin
        unique case (digits[{position, 2'b00} +: 4])
            4'h0: lit = 7'b0111111;
            4'h1: lit = 7'b0000110;
            4'h2: lit = 7'b1011011;
            4'h3: lit = 7'b1001111;
            4'h4: lit = 7'b1100110;
            4'h5: lit = 7'b1101101;
            4'h6: lit = 7'b1111101;
            4'h7: lit = 7'b0000111;
            4'h8: lit = 7'b1111111;
            4'h9: lit = 7'b1101111;
            4'ha: lit = 7'b1110111;
            4'hb: lit = 7'b1111100;
            4'hc: lit = 7'b0111001;
            4'hd: lit = 7'b1011110;
            4'he: lit = 7'b1111001;
            4'hf: lit = 7'b1110001;
        endcase
    end

    always_ff @(posedge clock) clocks <= clocks + 1'b1;
endmodule
