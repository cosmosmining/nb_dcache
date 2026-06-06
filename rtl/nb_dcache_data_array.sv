// ============================================================================
// nb_dcache_data_array.sv  -- SYNCHRONOUS-READ line data RAM (SRAM-mappable)
// ----------------------------------------------------------------------------
// One LINE_BITS-wide entry per (set, way), stored as one wide row per set
// (all WAYS side by side). Read and write are SYNCHRONOUS so the array infers as
// a single memory with one read port and one byte-write-enabled write port
// (i.e. a real byte-writable SRAM), instead of lowering to flip-flops.
//
// Read is WAY-PARALLEL: present rd_set with rd_en, and ALL ways of that set are
// available one cycle later, letting the tag compare pick the hit/victim way
// late (classic low-latency L1; in silicon, WAYS SRAM banks read in parallel).
//
//   cycle T   : drive rd_set, rd_en=1   (and/or a byte-strobed write)
//   cycle T+1 : rd_line[*] valid for the set presented at T
//
// The single flat write port (one address, one row-wide byte enable) is what
// keeps the memory cleanly inferable; the controller only ever writes the
// addressed way's bytes (others' enables are 0).
// ============================================================================
`include "nb_dcache_pkg.sv"

module nb_dcache_data_array
  import nb_dcache_pkg::*;
(
  input  logic       clk,

  // synchronous, way-parallel read
  input  logic       rd_en,
  input  set_t       rd_set,
  output line_t      rd_line [WAYS],   // registered; valid one cycle after rd_en

  // synchronous byte-strobed write (single way)
  input  logic       wr_en,
  input  set_t       wr_set,
  input  way_t       wr_way,
  input  line_t      wr_data,
  input  linestrb_t  wr_strb
);

  localparam int unsigned ROW_BITS  = WAYS * LINE_BITS;
  localparam int unsigned ROW_BYTES = WAYS * LINE_BYTES;

  // one wide row per set (all ways)
  logic [ROW_BITS-1:0] mem [NUM_SETS];

  // ---- synchronous read of the whole row -> all ways next cycle ------------
  logic [ROW_BITS-1:0] rd_row_q;
  always_ff @(posedge clk)
    if (rd_en)
      rd_row_q <= mem[rd_set];

  always_comb
    for (int w = 0; w < WAYS; w++)
      rd_line[w] = rd_row_q[w*LINE_BITS +: LINE_BITS];

  // ---- single byte-enabled write port -------------------------------------
  // Place the write word in the target way's slice and build a row-wide byte
  // enable that is non-zero only for that way's written bytes.
  logic [ROW_BITS-1:0]  wr_row;
  logic [ROW_BYTES-1:0] wr_ben;
  always_comb begin
    wr_row = '0;
    wr_ben = '0;
    wr_row[wr_way*LINE_BITS  +: LINE_BITS]  = wr_data;
    wr_ben[wr_way*LINE_BYTES +: LINE_BYTES] = wr_strb;
  end

  always_ff @(posedge clk)
    if (wr_en)
      for (int by = 0; by < ROW_BYTES; by++)
        if (wr_ben[by])
          mem[wr_set][by*8 +: 8] <= wr_row[by*8 +: 8];

endmodule
