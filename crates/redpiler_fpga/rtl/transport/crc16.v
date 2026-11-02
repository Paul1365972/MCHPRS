// CRC-16/CCITT-FALSE; a frame followed by its big-endian CRC leaves zero.
module crc16 (
    input wire [15:0] previous,
    input wire [7:0] data,
    output wire [15:0] next
);
    function [15:0] update(input [15:0] crc, input [7:0] byte_value);
        integer bit_index;
        begin
            update = crc ^ {byte_value, 8'b0};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                update = update[15] ? (update << 1) ^ 16'h1021 : update << 1;
        end
    endfunction

    assign next = update(previous, data);
endmodule
