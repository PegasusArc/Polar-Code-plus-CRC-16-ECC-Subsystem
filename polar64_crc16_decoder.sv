module polar64_crc16_decoder (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [63:0] rx,
    output logic        done,
    output logic [23:0] data_out,
    output logic        valid
);

  import polar_common_pkg::*;

  // Syndrome-based decoder: finds the minimum-weight codeword-domain error
  // pattern (weight 0–3) via parity-check column matching, corrects in the
  // u-domain, then validates with CRC-16.
  //
  // Pipeline (same 2-cycle latency as old BDD-3):
  //   Cycle start+1 : polar_transform64(rx) + syndrome extraction
  //   Cycle start+2 : syndrome match → correct → extract data/CRC → CRC check
  //   After start+2 : done=1, valid = found & crc_ok, data_out latched

  // ── Precomputed parity-check column syndromes ──────────────────────────
  // COL_SYN[j] = polar_transform64(1<<j) restricted to FROZEN_POS (24 bits).
  // Since dmin=8 >= 2*3+1, all weight-≤3 syndromes are unique.
  localparam logic [23:0] COL_SYN [64] = '{
    24'h000001, 24'h000003, 24'h000005, 24'h00000F,
    24'h000011, 24'h000033, 24'h000055, 24'h00007F,
    24'h000081, 24'h000183, 24'h000285, 24'h00038F,
    24'h000491, 24'h0005B3, 24'h0006D5, 24'h0007FF,
    24'h000801, 24'h001803, 24'h002805, 24'h00380F,
    24'h004811, 24'h005833, 24'h006855, 24'h00787F,
    24'h008881, 24'h009983, 24'h00AA85, 24'h00BB8F,
    24'h00CC91, 24'h00DDB3, 24'h00EED5, 24'h00FFFF,
    24'h010001, 24'h030003, 24'h050005, 24'h07000F,
    24'h090011, 24'h0B0033, 24'h0D0055, 24'h0F007F,
    24'h110081, 24'h130183, 24'h150285, 24'h17038F,
    24'h190491, 24'h1B05B3, 24'h1D06D5, 24'h1F07FF,
    24'h210801, 24'h231803, 24'h252805, 24'h27380F,
    24'h694811, 24'h6B5833, 24'h6D6855, 24'h6F787F,
    24'hB18881, 24'hB39983, 24'hB5AA85, 24'hB7BB8F,
    24'hF9CC91, 24'hFBDDB3, 24'hFDEED5, 24'hFFFFFF
  };

  // COL_G[j] = polar_transform64(1<<j), full 64-bit generator column.
  // Used to correct u_hat in the u-domain without a second polar transform.
  localparam logic [63:0] COL_G [64] = '{
    64'h0000000000000001, 64'h0000000000000003,
    64'h0000000000000005, 64'h000000000000000F,
    64'h0000000000000011, 64'h0000000000000033,
    64'h0000000000000055, 64'h00000000000000FF,
    64'h0000000000000101, 64'h0000000000000303,
    64'h0000000000000505, 64'h0000000000000F0F,
    64'h0000000000001111, 64'h0000000000003333,
    64'h0000000000005555, 64'h000000000000FFFF,
    64'h0000000000010001, 64'h0000000000030003,
    64'h0000000000050005, 64'h00000000000F000F,
    64'h0000000000110011, 64'h0000000000330033,
    64'h0000000000550055, 64'h0000000000FF00FF,
    64'h0000000001010101, 64'h0000000003030303,
    64'h0000000005050505, 64'h000000000F0F0F0F,
    64'h0000000011111111, 64'h0000000033333333,
    64'h0000000055555555, 64'h00000000FFFFFFFF,
    64'h0000000100000001, 64'h0000000300000003,
    64'h0000000500000005, 64'h0000000F0000000F,
    64'h0000001100000011, 64'h0000003300000033,
    64'h0000005500000055, 64'h000000FF000000FF,
    64'h0000010100000101, 64'h0000030300000303,
    64'h0000050500000505, 64'h00000F0F00000F0F,
    64'h0000111100001111, 64'h0000333300003333,
    64'h0000555500005555, 64'h0000FFFF0000FFFF,
    64'h0001000100010001, 64'h0003000300030003,
    64'h0005000500050005, 64'h000F000F000F000F,
    64'h0011001100110011, 64'h0033003300330033,
    64'h0055005500550055, 64'h00FF00FF00FF00FF,
    64'h0101010101010101, 64'h0303030303030303,
    64'h0505050505050505, 64'h0F0F0F0F0F0F0F0F,
    64'h1111111111111111, 64'h3333333333333333,
    64'h5555555555555555, 64'hFFFFFFFFFFFFFFFF
  };

  // ── Stage 1 combinational: polar transform + syndrome ──────────────────
  logic [63:0] u_hat_comb;
  logic [23:0] syn_comb;

  always_comb begin
    u_hat_comb = polar_transform64(rx);
    for (int k = 0; k < 24; k++)
      syn_comb[k] = u_hat_comb[FROZEN_POS[k]];
  end

  // ── Stage 1 pipeline registers ─────────────────────────────────────────
  logic [63:0] u_hat_s1;
  logic [23:0] syn_s1;
  logic        v1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      u_hat_s1 <= '0;
      syn_s1   <= '0;
      v1       <= 1'b0;
    end else begin
      v1       <= start;
      u_hat_s1 <= u_hat_comb;
      syn_s1   <= syn_comb;
    end
  end

  // ── Stage 2 combinational: syndrome match + correct + extract ──────────
  logic [63:0] u_err_comb;
  logic        found_comb;
  logic [63:0] u_corr_comb;
  logic [23:0] data_comb;
  logic [15:0] crc_rx_comb;

  always_comb begin
    u_err_comb = '0;
    found_comb = 1'b0;

    // Weight 0: no errors
    if (syn_s1 == 24'h0)
      found_comb = 1'b1;

    // Weight 1: single-bit error
    if (!found_comb) begin
      for (int j = 0; j < 64; j++) begin
        if (!found_comb && syn_s1 == COL_SYN[j]) begin
          found_comb = 1'b1;
          u_err_comb = COL_G[j];
        end
      end
    end

    // Weight 2: two-bit error
    if (!found_comb) begin
      for (int j1 = 0; j1 < 64; j1++) begin
        if (!found_comb) begin
          for (int j2 = j1 + 1; j2 < 64; j2++) begin
            if (!found_comb && syn_s1 == (COL_SYN[j1] ^ COL_SYN[j2])) begin
              found_comb = 1'b1;
              u_err_comb = COL_G[j1] ^ COL_G[j2];
            end
          end
        end
      end
    end

    // Weight 3: three-bit error
    if (!found_comb) begin
      for (int j1 = 0; j1 < 64; j1++) begin
        if (!found_comb) begin
          for (int j2 = j1 + 1; j2 < 64; j2++) begin
            if (!found_comb) begin
              for (int j3 = j2 + 1; j3 < 64; j3++) begin
                if (!found_comb && syn_s1 == (COL_SYN[j1] ^ COL_SYN[j2] ^ COL_SYN[j3])) begin
                  found_comb = 1'b1;
                  u_err_comb = COL_G[j1] ^ COL_G[j2] ^ COL_G[j3];
                end
              end
            end
          end
        end
      end
    end

    // Correct u_hat and extract data / CRC bits
    u_corr_comb = u_hat_s1 ^ u_err_comb;

    data_comb   = '0;
    crc_rx_comb = '0;
    for (int k = 0; k < 24; k++)
      data_comb[23-k] = u_corr_comb[INFO_POS[k]];
    for (int k = 0; k < 16; k++)
      crc_rx_comb[15-k] = u_corr_comb[INFO_POS[24+k]];
  end

  // ── Stage 2 pipeline registers ─────────────────────────────────────────
  logic [23:0] data_s2;
  logic [15:0] crc_rx_s2;
  logic        found_s2;
  logic        v2;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      data_s2   <= '0;
      crc_rx_s2 <= '0;
      found_s2  <= 1'b0;
      v2        <= 1'b0;
    end else begin
      v2        <= v1;
      data_s2   <= data_comb;
      crc_rx_s2 <= crc_rx_comb;
      found_s2  <= found_comb;
    end
  end

  // ── Output register: CRC gate + final decision ─────────────────────────
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done     <= 1'b0;
      valid    <= 1'b0;
      data_out <= '0;
    end else if (v2) begin
      done <= 1'b1;
      if (found_s2 && (crc16_ccitt24(data_s2) == crc_rx_s2)) begin
        valid    <= 1'b1;
        data_out <= data_s2;
      end else begin
        valid    <= 1'b0;
        data_out <= '0;
      end
    end else begin
      done     <= 1'b0;
      valid    <= 1'b0;
      data_out <= '0;
    end
  end

endmodule
