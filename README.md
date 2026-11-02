# Polar-Code-plus-CRC-16-ECC-Subsystem
**Project:** Mars Mission Error Correction Code (Polar N=64, K=40, CRC-16-CCITT)

---

## Quick Start — Vivado xsim

Run all commands from the `main/` directory (or from the project root by prefixing `main/`).

### 1. Compile

```bash
xvlog -sv \
  polar_common_pkg.sv \
  polar64_crc16_encoder.sv \
  polar64_crc16_decoder.sv \
  tb_basic.sv
```

### 2. Elaborate

```bash
xelab tb_basic -debug typical -s sim_snapshot
```

### 3. Simulate

```bash
xsim sim_snapshot -runall
```

### One-liner (PowerShell / bash)

```bash
xvlog -sv polar_common_pkg.sv polar64_crc16_encoder.sv polar64_crc16_decoder.sv tb_basic.sv && xelab tb_basic -debug typical -s sim_snapshot && xsim sim_snapshot -runall
```

All `$display` output goes to the console. A waveform dump (`tb_basic.vcd` or xsim default) is written when the simulation finishes.

---

## File Structure

```
main/
├── polar_common_pkg.sv       # shared parameters, tables, and functions
├── polar64_crc16_encoder.sv  # Polar + CRC-16 encoder
├── polar64_crc16_decoder.sv  # Polar BDD-3 + CRC-16 decoder
├── crc.sv                    # standalone CRC-16-CCITT reference (optional)
└── tb_basic.sv               # student-visible testbench
README.md                     # this file
report.md / report.pdf        # 2-page design report
```

---

## Module Descriptions

### `polar_common_pkg` — shared package

Imported by all RTL and testbench files.  Contains:

| Item | Description |
|------|-------------|
| `INFO_POS[40]` | Bit positions of the 40 information bits (24 data + 16 CRC) in the u-vector |
| `FROZEN_POS[24]` | Bit positions of the 24 frozen (always-0) parity bits |
| `crc16_ccitt24()` | Parallel XOR-tree CRC-16-CCITT for 24-bit data; O(1) combinational depth |
| `build_u()` | Assembles the 64-bit u-vector from data and CRC bits |
| `polar_transform64()` | 6-stage butterfly transform (no bit-reversal); self-inverse over GF(2) |
| `pos_tables_ok()` | Sanity check: verifies INFO_POS ∪ FROZEN_POS covers {0..63} exactly |
| `min_info_row_weight()` | Returns the minimum row weight of the generator sub-matrix (= 8 → d_min = 8) |

### `polar64_crc16_encoder` — encoder

| Port | Direction | Width | Description |
|------|-----------|-------|-------------|
| `clk` | input | 1 | clock |
| `rst_n` | input | 1 | active-low synchronous reset |
| `start` | input | 1 | 1-cycle pulse: latch `data_in` and begin encoding |
| `data_in` | input | 24 | plaintext payload |
| `done` | output | 1 | 1-cycle pulse: `codeword` is valid on this edge |
| `codeword` | output | 64 | transmitted polar codeword |

**Latency:** `done` asserts exactly **2 clock cycles** after `start` is sampled.  
**Throughput:** fully pipelined — a new `start` may be issued every cycle.

**Algorithm (combinational path):**
1. Compute CRC-16-CCITT over `data_in[23:0]` (MSB-first, no reflection).
2. Place data bits at `INFO_POS[0..23]` and CRC bits at `INFO_POS[24..39]` in u-vector; frozen positions = 0.
3. Apply `polar_transform64(u)` to produce `codeword`.

The entire path is a single combinational expression registered through a 2-FF pipeline.

### `polar64_crc16_decoder` — decoder

| Port | Direction | Width | Description |
|------|-----------|-------|-------------|
| `clk` | input | 1 | clock |
| `rst_n` | input | 1 | active-low synchronous reset |
| `start` | input | 1 | 1-cycle pulse: latch `rx` and begin decoding |
| `rx` | input | 64 | received (possibly corrupted) codeword |
| `done` | output | 1 | 1-cycle pulse: outputs are valid on this edge |
| `data_out` | output | 24 | decoded payload (valid only when `valid=1`) |
| `valid` | output | 1 | 1 = decoded message passes both BDD-3 and CRC; 0 = reject |

**Latency:** `done` asserts exactly **3 clock cycles** after `start` is sampled (≤ 8-cycle bonus threshold).  
**Throughput:** fully pipelined — a new `start` may be issued every cycle.

**Algorithm (3-stage pipeline):**

| Stage | Combinational work | Registered into |
|-------|--------------------|-----------------|
| 1 | `u_hat = polar_transform64(rx)` · extract syndrome from FROZEN_POS bits | `u_hat_s1`, `syn_s1` |
| 2 | Syndrome match (weight 0/1/2/3) → XOR correction vector · extract `data_comb`, `crc_rx_comb` | `data_s2`, `crc_rx_s2`, `found_s2` |
| 3 | Re-compute CRC on `data_s2`; compare to `crc_rx_s2`; gate `valid` | `data_out`, `valid`, `done` |

**Correction guarantee:** with d_min = 8, all 1–3 bit flips are corrected; all 4-bit patterns are detected (rejected). CRC provides a secondary integrity check.

---

## Code Parameters

| Parameter | Value |
|-----------|-------|
| Block length N | 64 |
| Information bits K | 40 (24 data + 16 CRC) |
| Frozen bits | 24 |
| Code rate R | 40/64 = 0.625 |
| Minimum distance d_min | 8 |
| Max correctable errors | 3 bits |
| Detectable (uncorrectable) errors | 4 bits |
| CRC polynomial | CRC-16-CCITT, G(x) = x¹⁶+x¹²+x⁵+1, `0x1021` |
| CRC init / reflection / XOR-out | 0x0000 / none / none |

---


## Known Limitations / Notes

- The encoder and decoder are fully pipelined but not designed for concurrent multi-word bursts (the latency pipeline assumes non-overlapping `start` pulses in the same cycle for simplest testbench integration; pipelined back-to-back is supported).
- The BDD-3 syndrome search uses fully unrolled combinational loops. This is synthesis-clean but produces large LUT fanout on FPGAs; timing closure at high frequencies may require retiming.
- `crc.sv` is an optional standalone reference module and is not required for simulation.
