// Shows numbers below 10,000 in full and larger ones truncated as 1.2E7 or, from 10^10, 1E12.
module number_glyphs (
    input wire [79:0] digits,
    output reg [31:0] glyphs
);
    localparam [7:0] BLANK = 8'h00, LETTER_E = 8'h79, POINT = 8'h80;

    function [7:0] numeral(input [3:0] digit);
        case (digit)
            4'd0: numeral = 8'h3f;
            4'd1: numeral = 8'h06;
            4'd2: numeral = 8'h5b;
            4'd3: numeral = 8'h4f;
            4'd4: numeral = 8'h66;
            4'd5: numeral = 8'h6d;
            4'd6: numeral = 8'h7d;
            4'd7: numeral = 8'h07;
            4'd8: numeral = 8'h7f;
            default: numeral = 8'h6f;
        endcase
    endfunction

    integer index;
    reg [4:0] leading;
    reg [3:0] exponent_tens, exponent_units;

    always @* begin
        leading = 0;
        for (index = 1; index < 20; index = index + 1)
            if (digits[index * 4 +: 4] != 0)
                leading = index;
        exponent_tens = leading >= 10 ? 4'd1 : 4'd0;
        exponent_units = leading >= 10 ? leading - 5'd10 : leading;
        if (leading < 4)
            glyphs = {
                leading >= 3 ? numeral(digits[15:12]) : BLANK,
                leading >= 2 ? numeral(digits[11:8]) : BLANK,
                leading >= 1 ? numeral(digits[7:4]) : BLANK,
                numeral(digits[3:0])
            };
        else if (leading < 10)
            glyphs = {
                numeral(digits[leading * 4 +: 4]) | POINT,
                numeral(digits[(leading - 1) * 4 +: 4]),
                LETTER_E,
                numeral(exponent_units)
            };
        else
            glyphs = {
                numeral(digits[leading * 4 +: 4]),
                LETTER_E,
                numeral(exponent_tens),
                numeral(exponent_units)
            };
    end
endmodule
