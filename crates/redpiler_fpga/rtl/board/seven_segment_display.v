// Each glyph is {decimal point, g, f, e, d, c, b, a}, lit when set; glyph 3 is the leftmost digit.
module seven_segment_display (
    input wire clock,
    input wire [31:0] glyphs,
    output wire [6:0] segment,
    output wire [3:0] anode,
    output wire decimal_point
);
    reg [17:0] refresh = 0;
    wire [1:0] digit = refresh[17:16];
    wire [7:0] glyph = glyphs[digit * 8 +: 8];

    always @(posedge clock)
        refresh <= refresh + 1;

    assign anode = ~(4'b0001 << digit);
    assign segment = ~glyph[6:0];
    assign decimal_point = ~glyph[7];
endmodule
