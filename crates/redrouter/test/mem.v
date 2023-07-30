module RamChip
    #(
		parameter AddressSize = 4,
    	parameter WordSize = 8,
    )
	(
    	input [AddressSize-1:0] address,
    	input [WordSize-1:0] data_in,
    	input clk, we, oe,
    	output [WordSize-1:0] data_out,
	);
    
    reg [WordSize-1:0] mem [0:(1 << AddressSize) - 1];
      
    always @(posedge clk) begin
        if (we)
            mem[address] <= data_in;
	end
    
    assign data_out = oe & !we ? mem[address] : { WordSize{1'b0} };
endmodule
