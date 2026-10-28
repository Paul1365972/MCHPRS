// One byte of CRC-16/CCITT-FALSE (polynomial 0x1021, start value 0xffff), the checksum of every
// frame in both directions.
module crc16 (
    input logic [15:0] checksum,
    input logic [7:0] data,
    output logic [15:0] next
);
    always_comb begin
        next = checksum ^ {data, 8'b0};
        for (int shifts = 0; shifts < 8; shifts++) begin
            if (next[15]) next = {next[14:0], 1'b0} ^ 16'h1021;
            else next = {next[14:0], 1'b0};
        end
    end
endmodule
