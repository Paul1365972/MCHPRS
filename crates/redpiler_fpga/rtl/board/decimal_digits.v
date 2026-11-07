// Converts one bit per clock (double dabble); digits holds 20 BCD digits, least significant first.
module decimal_digits (
    input wire clock,
    input wire start,
    input wire [63:0] value,
    output reg [79:0] digits = 0
);
    reg [63:0] remaining = 0;
    reg [79:0] converting = 0;
    reg [6:0] bits_left = 0;

    function [3:0] corrected_digit(input [3:0] digit);
        case (digit)
            4'd5: corrected_digit = 4'd8;
            4'd6: corrected_digit = 4'd9;
            4'd7: corrected_digit = 4'd10;
            4'd8: corrected_digit = 4'd11;
            4'd9: corrected_digit = 4'd12;
            default: corrected_digit = digit;
        endcase
    endfunction

    function [79:0] corrected(input [79:0] decimal);
        integer index;
        begin
            for (index = 0; index < 20; index = index + 1)
                corrected[index * 4 +: 4] = corrected_digit(decimal[index * 4 +: 4]);
        end
    endfunction

    wire [79:0] adjusted = corrected(converting);
    wire [79:0] shifted = {adjusted[78:0], remaining[63]};

    always @(posedge clock) begin
        if (bits_left != 0) begin
            converting <= shifted;
            remaining <= remaining << 1;
            bits_left <= bits_left - 1;
            if (bits_left == 1)
                digits <= shifted;
        end else if (start) begin
            converting <= 0;
            remaining <= value;
            bits_left <= 64;
        end
    end
endmodule
