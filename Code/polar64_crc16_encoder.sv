module polar64_crc16_encoder (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [23:0] data_in,
    output logic        done,
    output logic [63:0] codeword
);

  import polar_common_pkg::*;

  // ENC-5/ENC-6: fully combinational path through package functions.
  // No blocking assignments, no split butterfly, no intermediate registers.
  logic [63:0] codeword_comb;
  assign codeword_comb = polar_transform64(build_u(data_in, crc16_ccitt24(data_in)));

  // ENC-6: 2-FF delay line + 2-bit done shift register.
  // Timing: done asserts exactly 2 posedges after start is sampled high.
  // Pipelined throughput: 1 op per 2 cycles (start every 2nd cycle).
  // ENC-4: no busy lock needed — the pipeline drains naturally every 2 cycles.
  logic [63:0] pipe [1:0];
  logic [1:0]  done_pipe;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pipe[0]   <= '0;
      pipe[1]   <= '0;
      done_pipe <= 2'b00;
      codeword  <= '0;
      done      <= 1'b0;
    end else begin
      pipe[0]   <= codeword_comb;       // ENC-2: single non-blocking write
      pipe[1]   <= pipe[0];
      done_pipe <= {done_pipe[0], start};
      codeword  <= pipe[1];
      done      <= done_pipe[1];
    end
  end

endmodule
