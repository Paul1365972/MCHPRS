module binary_input #(
    parameter logic [pkg::NODE_BITS-1:0] INDEX,
    parameter logic POWERED
) (
    input logic clock,
    input logic taken_valid,
    input logic [pkg::NODE_BITS-1:0] taken_node,
    input pkg::drive_input_t taken_input,
    output logic powered = POWERED
);
    logic driven = 0;

    always_ff @(posedge clock) begin
        driven <= taken_valid && taken_node == INDEX;
        if (driven) begin
            if (taken_input.kind == pkg::INPUT_TOGGLE) powered <= !powered;
            else powered <= taken_input.value[0];
        end
    end
endmodule
