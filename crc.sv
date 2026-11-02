// Optional CRC module if not fully integrated into encoder/decoder
module crc16_ccitt (
    input  logic [23:0] data_in,
    output logic [15:0] crc_out
);

  import polar_common_pkg::*;

  assign crc_out = crc16_ccitt24(data_in);

endmodule
