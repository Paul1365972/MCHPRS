// A lever, or any other on/off node that the host drives as an input.
module lever #(
    parameter logic [23:0] INDEX,
    parameter logic POWERED
) (
    input logic clock,
    input logic taken_valid,
    input logic [23:0] taken_node,
    input pkg::drive_input_t taken_input,
    output logic powered = POWERED,
    output logic powered_next
);
    logic driven = 0;

    always_comb begin
        powered_next = powered;
        if (driven) begin
            if (taken_input.kind == pkg::INPUT_TOGGLE) powered_next = !powered;
            else powered_next = taken_input.value[0];
        end
    end

    always_ff @(posedge clock) begin
        driven <= taken_valid && taken_node == INDEX;
        powered <= powered_next;
    end
endmodule
