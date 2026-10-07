`default_nettype none

module delay_chain #(
    parameter WIDTH    = 8,
    parameter DEPTH    = 8
) (
    input logic              clk,
    input logic              rst,
    input logic [WIDTH-1:0]  din,
    output logic [WIDTH-1:0] dout
);
    reg [31:0] counter;
    reg delay_done;

    always_ff @(posedge clk) begin
        if (rst) begin
            counter <= 32'b0;
            delay_done <= 1'b0;
        end else begin
            if (counter < DEPTH) begin
                counter <= counter + 1'b1;
                delay_done <= 1'b0;
            end else begin
                delay_done <= 1'b1;
                dout <= din;
            end
        end
    end
endmodule