`timescale 1ns/1ps

package polar_common_pkg;

  // ── Code parameters ──────────────────────────────────────────────────────
  localparam int unsigned N       = 64;
  localparam int unsigned K       = 40;
  localparam int unsigned N_DATA  = 24;
  localparam int unsigned N_CRC   = 16;
  localparam int unsigned N_FROZ  = 24;

  // ── Bit-position tables ──────────────────────────────────────────────────
  // INFO_POS[0..23]  → data bit positions  (k=0  → data_in[23], MSB-first)
  // INFO_POS[24..39] → CRC  bit positions  (k=24 → crc[15],     MSB-first)
  // FROZEN_POS[0..23] → forced-zero positions
  // Selection criterion: 2^popcount(INFO_POS[k]) >= 8 for all k → d_min = 8.

  localparam int unsigned INFO_POS [40] = '{
    7,  11, 13, 14, 15, 19, 21, 22, 23,
    25, 26, 27, 28, 29, 30, 31,
    35, 37, 38, 39, 41, 42, 43, 44,
    45, 46, 47, 49, 50, 51, 53, 54,
    55, 57, 58, 59, 60, 61, 62, 63
  };

  localparam int unsigned FROZEN_POS [24] = '{
    0,  1,  2,  3,  4,  5,  6,  8,
    9,  10, 12, 16, 17, 18, 20, 24,
    32, 33, 34, 36, 40, 48, 52, 56
  };

  // ── CRC-16-CCITT (parallel XOR tree, O(1) combinational depth) ──────────
  // Poly x^16+x^12+x^5+1 (0x1021), MSB-first, init=0, no reflection/XOR-out.
  function automatic logic [15:0] crc16_ccitt24(input logic [23:0] din);
    logic [15:0] crc;
    crc[15] = din[3]  ^ din[7]  ^ din[10] ^ din[11] ^ din[18] ^ din[19] ^ din[21];
    crc[14] = din[2]  ^ din[6]  ^ din[9]  ^ din[10] ^ din[17] ^ din[18] ^ din[20];
    crc[13] = din[1]  ^ din[5]  ^ din[8]  ^ din[9]  ^ din[16] ^ din[17] ^ din[19] ^ din[23];
    crc[12] = din[0]  ^ din[4]  ^ din[7]  ^ din[8]  ^ din[15] ^ din[16] ^ din[18] ^ din[22] ^ din[23];
    crc[11] = din[6]  ^ din[10] ^ din[11] ^ din[14] ^ din[15] ^ din[17] ^ din[18] ^ din[19] ^ din[22] ^ din[23];
    crc[10] = din[5]  ^ din[9]  ^ din[10] ^ din[13] ^ din[14] ^ din[16] ^ din[17] ^ din[18] ^ din[21] ^ din[22];
    crc[ 9] = din[4]  ^ din[8]  ^ din[9]  ^ din[12] ^ din[13] ^ din[15] ^ din[16] ^ din[17] ^ din[20] ^ din[21] ^ din[23];
    crc[ 8] = din[3]  ^ din[7]  ^ din[8]  ^ din[11] ^ din[12] ^ din[14] ^ din[15] ^ din[16] ^ din[19] ^ din[20] ^ din[22] ^ din[23];
    crc[ 7] = din[2]  ^ din[6]  ^ din[7]  ^ din[10] ^ din[11] ^ din[13] ^ din[14] ^ din[15] ^ din[18] ^ din[19] ^ din[21] ^ din[22];
    crc[ 6] = din[1]  ^ din[5]  ^ din[6]  ^ din[9]  ^ din[10] ^ din[12] ^ din[13] ^ din[14] ^ din[17] ^ din[18] ^ din[20] ^ din[21] ^ din[23];
    crc[ 5] = din[0]  ^ din[4]  ^ din[5]  ^ din[8]  ^ din[9]  ^ din[11] ^ din[12] ^ din[13] ^ din[16] ^ din[17] ^ din[19] ^ din[20] ^ din[22];
    crc[ 4] = din[4]  ^ din[8]  ^ din[12] ^ din[15] ^ din[16] ^ din[23];
    crc[ 3] = din[3]  ^ din[7]  ^ din[11] ^ din[14] ^ din[15] ^ din[22] ^ din[23];
    crc[ 2] = din[2]  ^ din[6]  ^ din[10] ^ din[13] ^ din[14] ^ din[21] ^ din[22];
    crc[ 1] = din[1]  ^ din[5]  ^ din[9]  ^ din[12] ^ din[13] ^ din[20] ^ din[21] ^ din[23];
    crc[ 0] = din[0]  ^ din[4]  ^ din[8]  ^ din[11] ^ din[12] ^ din[19] ^ din[20] ^ din[22];
    return crc;
  endfunction

  // ── Build u vector ───────────────────────────────────────────────────────
  // Places 24 data bits at INFO_POS[0..23] and 16 CRC bits at INFO_POS[24..39].
  function automatic logic [63:0] build_u(
    input logic [23:0] din,
    input logic [15:0] crc
  );
    logic [63:0] u;
    u = '0;
    for (int i = 0; i < 24; i++)
      u[INFO_POS[i]] = din[23 - i];
    for (int i = 0; i < 16; i++)
      u[INFO_POS[24 + i]] = crc[15 - i];
    return u;
  endfunction

  // ── Polar butterfly transform ────────────────────────────────────────────
  // 6 stages, no bit-reversal. Self-inverse over GF(2).
  function automatic logic [63:0] polar_transform64(input logic [63:0] u);
    logic [63:0] v;
    int          step, half;
    v = u;
    for (int s = 0; s < 6; s++) begin
      step = 1 << (s + 1);
      half = 1 << s;
      for (int i = 0; i < 64; i += step)
        for (int j = 0; j < half; j++)
          v[i + j] = v[i + j] ^ v[i + j + half];
    end
    return v;
  endfunction

  // ── Table integrity check ─────────────────────────────────────────────────
  // Returns 1 iff INFO_POS ∪ FROZEN_POS = {0..63} with no overlaps.
  function automatic bit pos_tables_ok();
    bit [63:0] mask;
    mask = '0;
    for (int i = 0; i < 40; i++) begin
      if (mask[INFO_POS[i]]) return 1'b0;
      mask[INFO_POS[i]] = 1'b1;
    end
    for (int i = 0; i < 24; i++) begin
      if (mask[FROZEN_POS[i]]) return 1'b0;
      mask[FROZEN_POS[i]] = 1'b1;
    end
    return (mask == '1);
  endfunction

  // ── Minimum generator-matrix row weight ──────────────────────────────────
  // weight(row i) = 2^popcount(i); must be >= 8 for d_min >= 8.
  function automatic int min_info_row_weight();
    int min_w, w;
    min_w = 9999;
    for (int k = 0; k < 40; k++) begin
      w = 1 << $countones(INFO_POS[k]);
      if (w < min_w) min_w = w;
    end
    return min_w;
  endfunction

endpackage
