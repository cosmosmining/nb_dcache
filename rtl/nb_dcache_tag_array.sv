// ============================================================================
// nb_dcache_tag_array.sv  -- SYNCHRONOUS-READ tag / valid / dirty + pLRU
// ----------------------------------------------------------------------------
// Tag/valid/dirty state plus per-set tree-pLRU replacement state, with a
// SYNCHRONOUS (registered) read so its timing matches the data array and the
// tag storage can map to SRAM:
//
//   cycle T   : drive rd_set, rd_en=1
//   cycle T+1 : rd_tag/rd_valid/rd_dirty/rd_plru valid for that set (all ways)
//
// Writes (allocate / set-dirty / invalidate / pLRU touch) are synchronous; the
// controller issues reads in the address phase and writes in the decide/refill
// phase, so a read never collides with a write to the same set in the same cycle.
// Conflicting writes to the same field are resolved by the documented priority.
// ============================================================================
`include "nb_dcache_pkg.sv"

module nb_dcache_tag_array
  import nb_dcache_pkg::*;
(
  input  logic                 clk,
  input  logic                 rst_n,

  // --- synchronous set read (all ways) --------------------------------------
  input  logic                 rd_en,
  input  set_t                 rd_set,
  output tag_t                 rd_tag   [WAYS],
  output logic                 rd_valid [WAYS],
  output logic                 rd_dirty [WAYS],
  output logic [PLRU_BITS-1:0] rd_plru,

  // --- allocate a line into (set,way): sets valid, tag and initial dirty -----
  input  logic                 alloc_en,
  input  set_t                 alloc_set,
  input  way_t                 alloc_way,
  input  tag_t                 alloc_tag,
  input  logic                 alloc_dirty,

  // --- mark a way dirty (store hit) -----------------------------------------
  input  logic                 setdirty_en,
  input  set_t                 setdirty_set,
  input  way_t                 setdirty_way,

  // --- invalidate a way -----------------------------------------------------
  input  logic                 inv_en,
  input  set_t                 inv_set,
  input  way_t                 inv_way,

  // --- pLRU touch (on any hit/allocate) -------------------------------------
  input  logic                 plru_en,
  input  set_t                 plru_set,
  input  way_t                 plru_way
);

  // Metadata storage.
  tag_t                 tag_q   [NUM_SETS][WAYS];
  logic                 valid_q [NUM_SETS][WAYS];
  logic                 dirty_q [NUM_SETS][WAYS];
  logic [PLRU_BITS-1:0] plru_q  [NUM_SETS];

  // ---- synchronous read (registered outputs) -------------------------------
  always_ff @(posedge clk) begin
    if (rd_en) begin
      for (int w = 0; w < WAYS; w++) begin
        rd_tag[w]   <= tag_q[rd_set][w];
        rd_valid[w] <= valid_q[rd_set][w];
        rd_dirty[w] <= dirty_q[rd_set][w];
      end
      rd_plru <= plru_q[rd_set];
    end
  end

  // ---- valid / tag / dirty update ------------------------------------------
  // Priority for the dirty bit: allocate > set-dirty > invalidate.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int s = 0; s < NUM_SETS; s++)
        for (int w = 0; w < WAYS; w++) begin
          valid_q[s][w] <= 1'b0;
          dirty_q[s][w] <= 1'b0;
          tag_q[s][w]   <= '0;
        end
    end else begin
      if (alloc_en) begin
        valid_q[alloc_set][alloc_way] <= 1'b1;
        tag_q  [alloc_set][alloc_way] <= alloc_tag;
        dirty_q[alloc_set][alloc_way] <= alloc_dirty;
      end
      if (setdirty_en &&
          !(alloc_en && alloc_set == setdirty_set && alloc_way == setdirty_way))
        dirty_q[setdirty_set][setdirty_way] <= 1'b1;
      if (inv_en &&
          !(alloc_en && alloc_set == inv_set && alloc_way == inv_way)) begin
        valid_q[inv_set][inv_way] <= 1'b0;
        dirty_q[inv_set][inv_way] <= 1'b0;
      end
    end
  end

  // ---- pLRU update ----------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int s = 0; s < NUM_SETS; s++)
        plru_q[s] <= '0;
    end else if (plru_en) begin
      plru_q[plru_set] <= plru_update(plru_q[plru_set], plru_way);
    end
  end

endmodule
