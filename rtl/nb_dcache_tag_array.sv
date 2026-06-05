// ============================================================================
// nb_dcache_tag_array.sv
// ----------------------------------------------------------------------------
// Tag / valid / dirty state plus per-set tree-pLRU replacement state.
//
// One combinational read port exposes an entire set (all WAYS) so the top can
// do the parallel tag compare in the same cycle. Separate write ports update
// metadata; conflicts to the same field are resolved by the priority documented
// at each always_ff. Modeled as register arrays (small: NUM_LINES entries) so
// it elaborates cleanly under Verilator and Yosys without vendor SRAM macros.
// ============================================================================
`include "nb_dcache_pkg.sv"

module nb_dcache_tag_array
  import nb_dcache_pkg::*;
(
  input  logic                 clk,
  input  logic                 rst_n,

  // --- combinational set read (tag compare happens in the consumer) ---------
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

  // --- invalidate a way (e.g. external clear) -------------------------------
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

  // ---- combinational set read ----------------------------------------------
  always_comb begin
    for (int w = 0; w < WAYS; w++) begin
      rd_tag[w]   = tag_q[rd_set][w];
      rd_valid[w] = valid_q[rd_set][w];
      rd_dirty[w] = dirty_q[rd_set][w];
    end
    rd_plru = plru_q[rd_set];
  end

  // ---- valid / tag / dirty update ------------------------------------------
  // Priority for the dirty bit: allocate > set-dirty > invalidate. Allocate and
  // invalidate never target the same way in the same cycle by construction
  // (a refill and an external invalidate to the identical way are mutually
  // exclusive in the top's control), but the priority makes the intent explicit.
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
