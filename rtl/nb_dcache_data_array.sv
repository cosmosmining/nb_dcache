// ============================================================================
// nb_dcache_data_array.sv
// ----------------------------------------------------------------------------
// Line data storage, one LINE_BITS-wide entry per (set,way).
//
//  * One combinational read port returns a whole line; the top muxes out the
//    requested word and (on eviction) streams the line to the AXI engine.
//  * One write port with per-byte strobes covers both:
//      - store hit  : a single word's bytes enabled at the right offset
//      - line refill: all bytes enabled (store bytes pre-merged by the top so
//                     write-allocate lands atomically with the fill)
// ============================================================================
`include "nb_dcache_pkg.sv"

module nb_dcache_data_array
  import nb_dcache_pkg::*;
(
  input  logic       clk,

  // combinational line read
  input  set_t       rd_set,
  input  way_t       rd_way,
  output line_t      rd_line,

  // byte-strobed line write
  input  logic       wr_en,
  input  set_t       wr_set,
  input  way_t       wr_way,
  input  line_t      wr_data,
  input  linestrb_t  wr_strb
);

  line_t mem [NUM_SETS][WAYS];

  assign rd_line = mem[rd_set][rd_way];

  always_ff @(posedge clk) begin
    if (wr_en) begin
      for (int b = 0; b < LINE_BYTES; b++)
        if (wr_strb[b])
          mem[wr_set][wr_way][b*8 +: 8] <= wr_data[b*8 +: 8];
    end
  end

endmodule
