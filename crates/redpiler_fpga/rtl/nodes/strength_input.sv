module strength_input #(
    parameter logic [pkg::NODE_BITS-1:0] INDEX,
    parameter logic [3:0] STRENGTH
) (
    input logic clock,
    input logic taken_valid,
    input logic [pkg::NODE_BITS-1:0] taken_node,
    input pkg::drive_input_t taken_input,
    output logic [3:0] strength = STRENGTH
);
    logic driven = 0;

    always_ff @(posedge clock) begin
        driven <= taken_valid && taken_node == INDEX;
        if (driven && taken_input.kind == pkg::INPUT_SET) strength <= taken_input.value;
    end
endmodule
