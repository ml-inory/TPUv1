`default_nettype none

module PE #(
    parameter   ACT_WIDTH       = 8,
    parameter   WEIGHT_WIDTH    = 8,
    parameter   ACC_WIDTH       = 32
) (
    input logic clk,
    input logic rst,
    input logic load,
    input logic [WEIGHT_WIDTH-1:0]  w,
    input logic [ACT_WIDTH-1:0]     act,
    input logic [ACC_WIDTH-1:0]     psum_in,
    output logic [ACC_WIDTH-1:0]    psum_out,
    output logic valid
);
    reg [WEIGHT_WIDTH-1:0] w_s;
    logic signed [2*WEIGHT_WIDTH-1:0] mult;

    assign mult = $signed(w_s) * $signed(act);

    always_ff @( posedge clk ) begin 
        if (rst) begin
            w_s <= {WEIGHT_WIDTH{1'b0}};
            psum_out <= {ACC_WIDTH{1'b0}};
            valid <= 1'b0;
        end else if (load) begin
            w_s <= w;
            valid <= 1'b0;
        end else begin
            psum_out <= $signed(psum_in) + {{16{mult[15]}}, mult};
            valid <= 1'b1;
        end
    end
endmodule