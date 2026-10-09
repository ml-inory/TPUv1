`default_nettype none

// MXU 底部输出：
//   列0    列1    列2    列3
//   psum0  psum1  psum2  psum3
//     │      │      │      │
//     ▼      ▼      ▼      ▼
// ┌─────────────────────────────┐
// │        累加器阵列            │
// │                             │
// │  分块0: [c0][c1][c2][c3]    │  ← 第 1 段 K 的部分和
// │  分块1: [c0][c1][c2][c3]    │  ← 第 2 段 K 的部分和
// │                             │
// └─────────────────────────────┘
// 分块 0 存第 1 段 K 的所有列部分和
// 分块 1 存第 2 段 K 的所有列部分和
// 最后 c0 的最终结果 = 分块0[c0] + 分块1[c0]
module Accumulator #(
    parameter WIDTH     = 32,
    parameter COL       = 256,
    parameter BLOCK_NUM = 16
) (
    input  logic clk,
    input  logic rst,
    input  logic psum_valid,
    input  logic [WIDTH-1:0] psum [0:COL-1],
    output logic [WIDTH-1:0] dout [0:COL-1]
);
    logic active_group;   // 在写的组，另一组进入汇总
    reg [$clog2(BLOCK_NUM)-1:0] block_idx;
    integer i, j;

    // 双缓冲
    reg [WIDTH-1:0] acc [0:BLOCK_NUM-1][0:COL-1];

    always_comb begin
        active_group = (block_idx >= BLOCK_NUM / 2);
        for (j = 0; j < COL; j++) begin
            dout[j] = '0;
            for (int s = 0; s < BLOCK_NUM / 2; s++)
                dout[j] = dout[j] + acc[active_group ? s : (BLOCK_NUM / 2 + s)][j];
        end
    end
    
    always_ff @(posedge clk) begin
        if (rst) begin
            block_idx <= '0;
            for (i = 0; i < BLOCK_NUM; i++) begin
                for (j = 0; j < COL; j++) begin
                    acc[i][j] <= {WIDTH{1'b0}};
                end
            end
        end else begin
            if (psum_valid) begin
                for (i = 0; i < COL; i++) begin
                    acc[block_idx][i] <= psum[i];
                end
                block_idx <= (block_idx == BLOCK_NUM - 1) ? '0 : block_idx + 1'b1;
            end
        end
    end
endmodule