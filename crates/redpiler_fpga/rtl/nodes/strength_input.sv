module strength_input #(
    parameter logic [pkg::NODE_BITS-1:0] INDEX,
    parameter logic [3:0] STRENGTH
) (
    input logic clock,
    input logic taken_valid,
    input logic [pkg::NODE_BITS-1:0] taken_node,
    input pkg::drive_input_t taken_input,
    output logic [3:0] strength = STRENGTH,
    output logic [3:0] strength_next
);
    logic driven = 0;

    always_comb begin
        strength_next = strength;
        if (driven && taken_input.kind == pkg::INPUT_SET) strength_next = taken_input.value;
    end

    always_ff @(posedge clock) begin
        driven <= taken_valid && taken_node == INDEX;
        strength <= strength_next;
    end
endmodule
