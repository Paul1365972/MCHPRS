module seven_segment_display (
    input wire clock,
    input wire [15:0] value,
    output reg [6:0] segment,
    output wire [3:0] anode,
    output wire decimal_point
);
    reg [17:0] refresh = 0;
    wire [1:0] digit = refresh[17:16];
    wire [3:0] numeral = value[digit * 4 +: 4];

    always @(posedge clock)
        refresh <= refresh + 1;

    assign anode = ~(4'b0001 << digit);
    assign decimal_point = 1;

    always @* begin
        case (numeral)
            4'h0: segment = 7'b1000000;
            4'h1: segment = 7'b1111001;
            4'h2: segment = 7'b0100100;
            4'h3: segment = 7'b0110000;
            4'h4: segment = 7'b0011001;
            4'h5: segment = 7'b0010010;
            4'h6: segment = 7'b0000010;
            4'h7: segment = 7'b1111000;
            4'h8: segment = 7'b0000000;
            4'h9: segment = 7'b0010000;
            4'ha: segment = 7'b0001000;
            4'hb: segment = 7'b0000011;
            4'hc: segment = 7'b1000110;
            4'hd: segment = 7'b0100001;
            4'he: segment = 7'b0000110;
            4'hf: segment = 7'b0001110;
        endcase
    end
endmodule
