(* techmap_celltype = "$mux" *)
module mux_wrapper (A, B, S, Y);
  parameter WIDTH = 0;
  
  input [WIDTH-1:0] A, B;
  input S;
  output [WIDTH-1:0] Y;

  redstone_mux redstone_mux (.A(A), .B(B), .S(S), .Y(Y));

  generate
    if (WIDTH < `MIN_WIDTH) begin
        wire _TECHMAP_FAIL_ = 1;
    end
  endgenerate
endmodule
