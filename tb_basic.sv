`timescale 1ns/1ps

// ============================================================
// polar_common_pkg — shared constants, tables, and functions
// Embedded here so tb_basic.sv is the single source of truth.
// Compile this file FIRST so encoder/decoder can import it:
//   xvlog -sv tb_basic.sv polar64_crc16_encoder.sv polar64_crc16_decoder.sv
// ============================================================
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
  // Equations derived from the 16×24 GF(2) linear-map matrix; verified against
  // serial reference for 0x000000, 0xFFFFFF, 0xABCDEF, 0x123456, 0xDEAD42.
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

// ============================================================
// tb_basic — smoke + extended regression testbench
// ============================================================
// Directed checks (public):
//   Case A: 0 flips   -> valid=1, correct data_out
//   Case B: 1-3 flips -> valid=1, correct data_out
//   Case C: 4 flips   -> valid=0
// Interface timing:
//   Encoder done exactly +2 cycles after start (1-cycle pulse)
//   Decoder done within 12 cycles after start  (1-cycle pulse)
// Extended regression (compile with +define+EXTENDED):
//   100 random payloads × {0,1,2,3,4,5 flips}, back-to-back, mid-reset
// ============================================================

module tb_basic;

  import polar_common_pkg::*;

  logic clk;
  logic rst_n;

  // Encoder DUT ports
  logic        enc_start, enc_done;
  logic [23:0] data_in;
  logic [63:0] codeword;

  // Decoder DUT ports
  logic        dec_start, dec_done;
  logic [63:0] rx;
  logic [23:0] data_out;
  logic        valid;

  // Test signals
  logic [23:0] din;
  logic [63:0] cw_ref, cw_dut;
  logic [23:0] dout;
  logic        v;
  int unsigned lat;

  // Flags (declared at module level for VCS compatibility)
  logic enc_timing_ok;
  logic dec_timing_ok;
  bit   timing_ok_all;
  bit   enc_ok;
  bit   dec_ok;

  // Smoke scoring
  int smoke_score;
  int smoke_total;
  int smoke_fail;

  // Extended-test scratch variables
  logic [31:0] ext_seed;
  int          ext_pass, ext_fail;
  int          fp0, fp1, fp2;

  // ── DUT instantiations ──────────────────────────────────────────────────
  polar64_crc16_encoder u_enc (
    .clk(clk), .rst_n(rst_n),
    .start(enc_start), .data_in(data_in),
    .done(enc_done), .codeword(codeword)
  );

  polar64_crc16_decoder u_dec (
    .clk(clk), .rst_n(rst_n),
    .start(dec_start), .rx(rx),
    .done(dec_done), .data_out(data_out), .valid(valid)
  );

  // 100 MHz clock
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ── Testbench-only reference functions ──────────────────────────────────

  function automatic logic [63:0] ref_encode(input logic [23:0] d_in);
    logic [15:0] crc;
    logic [63:0] u;
    crc = crc16_ccitt24(d_in);
    u   = build_u(d_in, crc);
    return polar_transform64(u);
  endfunction

  function automatic logic [63:0] bit_mask(input int unsigned b);
    logic [63:0] m;
    m    = 64'b0;
    m[b] = 1'b1;
    return m;
  endfunction

  // ── Helper tasks ────────────────────────────────────────────────────────

  task automatic pulse_start(ref logic s);
    @(negedge clk); s = 1'b1;
    @(negedge clk); s = 1'b0;
  endtask

  // data_in is assigned BEFORE pulse_start so it is stable before the
  // posedge that samples enc_start — ordering is intentional (TB-2).
  task automatic do_encode(
    input  logic [23:0] d_in,
    output logic [63:0] cw,
    output logic        timing_ok
  );
    timing_ok = 1'b1;
    data_in   = d_in;
    pulse_start(enc_start);
    @(posedge clk); @(negedge clk);
    if (enc_done) timing_ok = 1'b0;          // +1: must still be 0
    @(posedge clk); @(negedge clk);
    if (!enc_done) timing_ok = 1'b0;         // +2: must be 1
    cw = codeword;
    @(posedge clk); @(negedge clk);
    if (enc_done) timing_ok = 1'b0;          // +3: must be 0 again
  endtask

  task automatic do_decode(
    input  logic [63:0] rx_in,
    output logic [23:0] d_out,
    output logic        vld,
    output int unsigned latency,
    output logic        timing_ok
  );
    timing_ok = 1'b1;
    latency   = 0;
    rx        = rx_in;
    pulse_start(dec_start);
    while (1) begin
      @(posedge clk); latency++;
      @(negedge clk);
      if (dec_done) break;
      if (latency >= 12) begin timing_ok = 1'b0; break; end
    end
    d_out = data_out;
    vld   = valid;
    if (timing_ok) begin
      @(posedge clk); @(negedge clk);
      if (dec_done) timing_ok = 1'b0;        // done must be 1-cycle pulse
    end
  endtask

  task automatic smoke_item(input string name, input int pts, input bit pass);
    if (pass) begin
      smoke_score += pts;
      $display("[SMOKE][PASS] +%0d : %s", pts, name);
    end else begin
      smoke_fail++;
      $display("[SMOKE][FAIL] +%0d : %s", pts, name);
    end
  endtask

  // ── Main test sequence ──────────────────────────────────────────────────
  initial begin
    enc_start = 1'b0; dec_start = 1'b0;
    data_in = '0; rx = '0;
    smoke_score = 0; smoke_fail = 0; smoke_total = 30;

    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    $display("============================================================");
    $display(" tb_basic (tbl2026.8) : PUBLIC smoke checks (NOT full grading)");
    $display("============================================================");
    $display("[TB] pos_tables_ok=%0d, min_info_row_weight=%0d (target >= 8)",
             pos_tables_ok(), min_info_row_weight());

    din    = 24'hABCDEF;
    cw_ref = ref_encode(din);

    // ── Encoder smoke ──────────────────────────────────────────────────
    do_encode(din, cw_dut, enc_timing_ok);
    enc_ok = pos_tables_ok() && enc_timing_ok && (cw_dut === cw_ref);
    smoke_item("Encoder: matches reference on 24'hABCDEF and done @ +2", 10, enc_ok);

    // ── Decoder smoke (Case A / B / C) ─────────────────────────────────
    timing_ok_all = enc_timing_ok;
    dec_ok        = 1'b1;

    do_decode(cw_ref, dout, v, lat, dec_timing_ok);
    timing_ok_all &= dec_timing_ok;
    dec_ok &= (v === 1'b1) && (dout === din);

    do_decode(cw_ref ^ bit_mask(5), dout, v, lat, dec_timing_ok);
    timing_ok_all &= dec_timing_ok;
    dec_ok &= (v === 1'b1) && (dout === din);

    do_decode(cw_ref ^ bit_mask(0) ^ bit_mask(63), dout, v, lat, dec_timing_ok);
    timing_ok_all &= dec_timing_ok;
    dec_ok &= (v === 1'b1) && (dout === din);

    do_decode(cw_ref ^ bit_mask(0) ^ bit_mask(1) ^ bit_mask(63), dout, v, lat, dec_timing_ok);
    timing_ok_all &= dec_timing_ok;
    dec_ok &= (v === 1'b1) && (dout === din);

    do_decode(cw_ref ^ bit_mask(0) ^ bit_mask(1) ^ bit_mask(2) ^ bit_mask(3),
              dout, v, lat, dec_timing_ok);
    timing_ok_all &= dec_timing_ok;
    dec_ok &= (v === 1'b0);

    do_decode(cw_ref ^ bit_mask(0) ^ bit_mask(1) ^ bit_mask(2) ^ bit_mask(3) ^ bit_mask(4),
              dout, v, lat, dec_timing_ok);
    timing_ok_all &= dec_timing_ok;
    dec_ok &= !((v === 1'b1) && (dout !== din));

    smoke_item("Decoder: Case A/B/C on ABCDEF (plus 1 fail-safe spot check)", 10, dec_ok);
    smoke_item("Interface timing: ENC done @+2, DEC done <=12, pulses are 1-cycle",
               10, timing_ok_all);

    $display("------------------------------------------------------------");
    $display("[SUMMARY] SMOKE score = %0d / %0d", smoke_score, smoke_total);
    $display("------------------------------------------------------------");

    if (smoke_fail != 0) begin
      $display("[tb_basic] FAIL");
      $fatal(1);
    end else begin
      $display("[tb_basic] PASS");
      $display("[NOTE] Passing tb_basic does NOT guarantee full credit.");
      $display("[NOTE] Completion requires BASE SCORE 100/100 in tb_hidden.");
    end

    // ── Extended regression (compile with +define+EXTENDED) ────────────
`ifdef EXTENDED
    ext_seed = 32'hDEAD_BEEF;
    ext_pass = 0; ext_fail = 0;
    $display("============================================================");
    $display(" EXTENDED regression: 100 random payloads");
    $display("============================================================");

    for (int vi = 0; vi < 100; vi++) begin
      ext_seed = ext_seed * 32'd1664525 + 32'd1013904223;
      din    = ext_seed[23:0];
      cw_ref = ref_encode(din);

      do_encode(din, cw_dut, enc_timing_ok);
      if (!enc_timing_ok || cw_dut !== cw_ref) begin
        $display("[EXT][FAIL] vi=%0d enc din=0x%06X", vi, din); ext_fail++;
      end

      // 0 flips
      do_decode(cw_ref, dout, v, lat, dec_timing_ok);
      if (!dec_timing_ok || v !== 1'b1 || dout !== din) begin
        $display("[EXT][FAIL] vi=%0d 0-flip valid=%0b dout=0x%06X", vi, v, dout); ext_fail++;
      end else ext_pass++;

      // 1 random flip
      ext_seed = ext_seed * 32'd1664525 + 32'd1013904223;
      fp0 = ext_seed[5:0];
      do_decode(cw_ref ^ bit_mask(fp0), dout, v, lat, dec_timing_ok);
      if (!dec_timing_ok || v !== 1'b1 || dout !== din) begin
        $display("[EXT][FAIL] vi=%0d 1-flip bit=%0d valid=%0b", vi, fp0, v); ext_fail++;
      end else ext_pass++;

      // 2 distinct random flips
      ext_seed = ext_seed * 32'd1664525 + 32'd1013904223;
      fp1 = ext_seed[5:0];
      if (fp1 == fp0) fp1 = (fp0 + 1) % 64;
      do_decode(cw_ref ^ bit_mask(fp0) ^ bit_mask(fp1), dout, v, lat, dec_timing_ok);
      if (!dec_timing_ok || v !== 1'b1 || dout !== din) begin
        $display("[EXT][FAIL] vi=%0d 2-flip valid=%0b", vi, v); ext_fail++;
      end else ext_pass++;

      // 3 distinct random flips
      ext_seed = ext_seed * 32'd1664525 + 32'd1013904223;
      fp2 = ext_seed[5:0];
      if (fp2 == fp0) fp2 = (fp2 + 1) % 64;
      if (fp2 == fp1) fp2 = (fp2 + 1) % 64;
      if (fp2 >= 64)  fp2 = fp2 % 64;
      if (fp2 == fp0) fp2 = (fp2 + 2) % 64;
      do_decode(cw_ref ^ bit_mask(fp0) ^ bit_mask(fp1) ^ bit_mask(fp2),
                dout, v, lat, dec_timing_ok);
      if (!dec_timing_ok || v !== 1'b1 || dout !== din) begin
        $display("[EXT][FAIL] vi=%0d 3-flip valid=%0b", vi, v); ext_fail++;
      end else ext_pass++;

      // 4 flips — must reject
      do_decode(cw_ref ^ 64'hF, dout, v, lat, dec_timing_ok);
      if (!dec_timing_ok || v !== 1'b0) begin
        $display("[EXT][FAIL] vi=%0d 4-flip not rejected valid=%0b", vi, v); ext_fail++;
      end else ext_pass++;

      // 5 flips — fail-safe: if valid=1, data must be correct
      do_decode(cw_ref ^ 64'h1F, dout, v, lat, dec_timing_ok);
      if ((v === 1'b1) && (dout !== din)) begin
        $display("[EXT][FAIL] vi=%0d fail-safe VIOLATION valid=1 dout=0x%06X exp=0x%06X",
                 vi, dout, din); ext_fail++;
      end else ext_pass++;
    end

    // Back-to-back encode (1-cycle gap between start pulses)
    $display("[EXTENDED] Back-to-back encode...");
    begin
      logic [63:0] cw1_exp, cw2_exp, cw1_got, cw2_got;
      cw1_exp = ref_encode(24'hABCDEF);
      cw2_exp = ref_encode(24'h123456);
      data_in = 24'hABCDEF;
      @(negedge clk); enc_start = 1'b1;
      @(negedge clk); enc_start = 1'b0;
      @(negedge clk);                          // 1-cycle gap
      data_in = 24'h123456;
      @(negedge clk); enc_start = 1'b1;
      @(negedge clk); enc_start = 1'b0;
      repeat (2) @(posedge clk); @(negedge clk); cw1_got = codeword;
      repeat (2) @(posedge clk); @(negedge clk); cw2_got = codeword;
      if (cw1_got !== cw1_exp || cw2_got !== cw2_exp) begin
        $display("[EXT][FAIL] back-to-back cw1=%h(exp %h) cw2=%h(exp %h)",
                 cw1_got, cw1_exp, cw2_got, cw2_exp); ext_fail++;
      end else begin
        $display("[EXT][PASS] back-to-back encode OK"); ext_pass++;
      end
    end

    // Mid-stream reset
    $display("[EXTENDED] Mid-stream reset...");
    data_in = 24'hDEADBE;
    @(negedge clk); enc_start = 1'b1;
    @(negedge clk); enc_start = 1'b0;
    @(negedge clk); rst_n = 1'b0;
    repeat (3) @(posedge clk);
    if (enc_done !== 1'b0 || dec_done !== 1'b0 || valid !== 1'b0) begin
      $display("[EXT][FAIL] reset did not clear outputs"); ext_fail++;
    end else begin
      $display("[EXT][PASS] mid-stream reset OK"); ext_pass++;
    end
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    $display("------------------------------------------------------------");
    $display("[EXTENDED] %0d pass, %0d fail", ext_pass, ext_fail);
    $display("------------------------------------------------------------");
    if (ext_fail > 0) begin $display("[EXTENDED] FAIL"); $fatal(1); end
    else $display("[EXTENDED] PASS");
`endif

    $finish;
  end

endmodule
