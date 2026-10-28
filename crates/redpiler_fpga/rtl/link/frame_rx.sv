module frame_rx #(
    parameter int CONTENT_BYTES = 1 + pkg::ARGUMENT_BYTES
) (
    input logic clock,
    input logic valid,
    input logic [7:0] data,
    output logic completed = 0,
    output pkg::status_t status = pkg::OK,
    output logic [CONTENT_BYTES * 8 - 1:0] content
);
    localparam int LENGTH_BITS = $clog2(CONTENT_BYTES + 2);

    logic [CONTENT_BYTES * 8 - 1:0] bytes = 0;
    logic [LENGTH_BITS - 1:0] length = 0;
    logic escaping = 0;
    logic misescaped = 0;
    logic [7:0] unstuffed;
    pkg::status_t frame_status;

    assign content = bytes;

    always_comb begin
        if (!escaping) unstuffed = data;
        else if (data == pkg::ESCAPED_END) unstuffed = pkg::FRAME_END;
        else unstuffed = pkg::ESCAPE;
    end

    always_comb begin
        if (misescaped || escaping) frame_status = pkg::BAD_ESCAPE;
        else if (length != LENGTH_BITS'(CONTENT_BYTES)) frame_status = pkg::BAD_LENGTH;
        else frame_status = pkg::OK;
    end

    always_ff @(posedge clock) begin
        completed <= 0;
        if (valid && data == pkg::FRAME_END) begin
            completed <= length != 0 || escaping;
            status <= frame_status;
            length <= 0;
            escaping <= 0;
            misescaped <= 0;
        end else if (valid && data == pkg::ESCAPE && !escaping) begin
            escaping <= 1;
        end else if (valid) begin
            escaping <= 0;
            if (escaping && data != pkg::ESCAPED_END && data != pkg::ESCAPED_ESCAPE) misescaped <= 1;
            if (length <= LENGTH_BITS'(CONTENT_BYTES)) length <= length + 1'b1;
            bytes <= {unstuffed, bytes[CONTENT_BYTES * 8 - 1:8]};
        end
    end
endmodule
