`default_nettype none

module PE #(
    parameter   ACT_WIDTH       = 8,
    parameter   WEIGHT_WIDTH    = 8,
    parameter   ACC_WIDTH       = 32
) (
    input logic clk,
    input logic rst,
    // weight
    input logic                     load_weight,
    input logic [WEIGHT_WIDTH-1:0]  weight_in,
    // act
    input logic                     act_in_valid,
    input logic [ACT_WIDTH-1:0]     act_in,
    output logic                    act_out_valid,
    output logic [ACT_WIDTH-1:0]    act_out,
    // psum
    input logic [ACC_WIDTH-1:0]     psum_in,
    output logic [ACC_WIDTH-1:0]    psum_out,
    output logic                    psum_out_valid
);
    reg [WEIGHT_WIDTH-1:0] weight_reg;
    logic signed [2*WEIGHT_WIDTH-1:0] mult;

    assign mult = $signed(weight_reg) * $signed(act_in);

    always_ff @( posedge clk ) begin 
        if (rst) begin
            weight_reg <= {WEIGHT_WIDTH{1'b0}};
            psum_out_valid <= 1'b0;
            act_out_valid <= 1'b0;
        end else if (load_weight) begin
            weight_reg <= weight_in;
            psum_out_valid <= 1'b0;
            act_out_valid <= 1'b0;
        end else begin
            if (act_in_valid) begin
                psum_out <= $signed(psum_in) + {{ACC_WIDTH-2*WEIGHT_WIDTH{mult[15]}}, mult};
                act_out <= act_in;
            end
            psum_out_valid <= act_in_valid;
            act_out_valid <= act_in_valid;
        end
    end
endmodule