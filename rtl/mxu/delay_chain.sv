`default_nettype none

// DEPTH-stage shift register delay line.
//   * dout is din delayed by exactly DEPTH clock cycles
//     (DEPTH = 0 -> dout = din, a pure wire)
//   * rst clears every stage, so dout is 0 while the line is empty
// Because the delay is continuous, feeding the same vector to all rows of a
// systolic array lets the array skew its rows internally (row i sees it i
// cycles later) - see rtl/mxu/mxu.sv.
module delay_chain #(
    parameter WIDTH    = 8,
    parameter DEPTH    = 8
) (
    input logic              clk,
    input logic              rst,
    input logic [WIDTH-1:0]  din,
    output logic [WIDTH-1:0] dout
);
    generate
        if (DEPTH == 0) begin : g_pass
            assign dout = din;
        end else begin : g_delay
            logic [WIDTH-1:0] chains [0:DEPTH-1];

            always_ff @(posedge clk) begin
                if (rst) begin
                    for (int s = 0; s < DEPTH; s++) chains[s] <= {WIDTH{1'b0}};
                end else begin
                    chains[0] <= din;
                    for (int s = 1; s < DEPTH; s++) chains[s] <= chains[s-1];
                end
            end

            assign dout = chains[DEPTH-1];
        end
    endgenerate
endmodule

`default_nettype wire
