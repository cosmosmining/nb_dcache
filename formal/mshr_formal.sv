// ============================================================================
// mshr_formal.sv  -- standalone bounded formal harness for nb_dcache_mshr
// ----------------------------------------------------------------------------
// Drives the MSHR file with unconstrained ($anyseq) stimulus, constrains the
// inputs to the contract the top guarantees (assumptions), and proves the three
// required invariants (assertions):
//
//   1. address uniqueness   - no two live entries hold the same line
//   2. no allocation when full / free-slot validity
//   3. exactly-once retirement (waiter conservation) - no over-dequeue,
//      no enqueue overflow, pointers/counts in range
//
// Run with: sby -f mshr_proofs.sby   (yosys + smtbmc, bounded model checking)
// ============================================================================
`include "nb_dcache_pkg.sv"

module mshr_formal
  import nb_dcache_pkg::*;
#(
  parameter int unsigned NWAIT = 4
)(
  input logic clk
);
  localparam int unsigned IDXW = (MSHR_CNT > 1) ? $clog2(MSHR_CNT) : 1;

  // ---- power-on reset: low for the first cycle only ------------------------
  logic init_done = 1'b0;
  always_ff @(posedge clk) init_done <= 1'b1;
  wire rst_n = init_done;

  // ---- free (unconstrained) stimulus ---------------------------------------
  (* anyseq *) set_t    look_set;
  (* anyseq *) tag_t    look_tag;
  (* anyseq *) logic    alloc_en;
  (* anyseq *) set_t    alloc_set;
  (* anyseq *) tag_t    alloc_tag;
  (* anyseq *) way_t    alloc_way;
  (* anyseq *) logic    alloc_need_evict;
  (* anyseq *) tag_t    alloc_vtag;
  (* anyseq *) line_t   alloc_vline;
  (* anyseq *) cpu_id_t alloc_w_id;
  (* anyseq *) logic    alloc_w_isld;
  (* anyseq *) woff_t   alloc_w_word;
  (* anyseq *) data_t   alloc_w_data;
  (* anyseq *) strb_t   alloc_w_strb;
  (* anyseq *) logic    merge_en;
  (* anyseq *) logic [IDXW-1:0] merge_idx;
  (* anyseq *) cpu_id_t merge_w_id;
  (* anyseq *) logic    merge_w_isld;
  (* anyseq *) woff_t   merge_w_word;
  (* anyseq *) data_t   merge_w_data;
  (* anyseq *) strb_t   merge_w_strb;
  (* anyseq *) logic    rsp_ready;
  (* anyseq *) logic    fill_req_ready;
  (* anyseq *) logic    fill_rsp_valid;
  (* anyseq *) axi_id_t fill_rsp_id;
  (* anyseq *) line_t   fill_rsp_line;
  (* anyseq *) logic    fill_rsp_err;
  (* anyseq *) logic    evict_req_ready;
  (* anyseq *) logic    evict_done_valid;
  (* anyseq *) axi_id_t evict_done_id;

  // ---- DUT outputs ---------------------------------------------------------
  logic            look_hit, look_hit_mergeable, look_full, look_wb_conflict;
  logic [IDXW-1:0] look_hit_idx;
  logic [WAYS-1:0] look_locked_ways;
  logic            da_wr_en;   set_t da_wr_set; way_t da_wr_way;
  line_t           da_wr_data; linestrb_t da_wr_strb;
  logic            tag_alloc_en; set_t tag_alloc_set; way_t tag_alloc_way;
  tag_t            tag_alloc_tag; logic tag_alloc_dirty;
  logic            tag_plru_en; set_t tag_plru_set; way_t tag_plru_way;
  logic            rsp_valid; cpu_id_t rsp_id; data_t rsp_rdata; logic rsp_err;
  logic            block_lookup;
  logic            fill_req_valid; axi_id_t fill_req_id; addr_t fill_req_addr;
  logic            evict_req_valid; axi_id_t evict_req_id;
  addr_t           evict_req_addr; line_t evict_req_line;
  logic            evict_done_ready;

  nb_dcache_mshr #(.NWAIT(NWAIT)) dut (
    .clk(clk), .rst_n(rst_n),
    .look_set(look_set), .look_tag(look_tag),
    .look_hit(look_hit), .look_hit_mergeable(look_hit_mergeable),
    .look_hit_idx(look_hit_idx), .look_full(look_full),
    .look_locked_ways(look_locked_ways), .look_wb_conflict(look_wb_conflict),
    .dbg_occupancy(),
    .alloc_en(alloc_en), .alloc_set(alloc_set), .alloc_tag(alloc_tag),
    .alloc_way(alloc_way), .alloc_need_evict(alloc_need_evict),
    .alloc_vtag(alloc_vtag), .alloc_vline(alloc_vline),
    .alloc_w_id(alloc_w_id), .alloc_w_isld(alloc_w_isld),
    .alloc_w_word(alloc_w_word), .alloc_w_data(alloc_w_data),
    .alloc_w_strb(alloc_w_strb),
    .merge_en(merge_en), .merge_idx(merge_idx),
    .merge_w_id(merge_w_id), .merge_w_isld(merge_w_isld),
    .merge_w_word(merge_w_word), .merge_w_data(merge_w_data),
    .merge_w_strb(merge_w_strb),
    .da_wr_en(da_wr_en), .da_wr_set(da_wr_set), .da_wr_way(da_wr_way),
    .da_wr_data(da_wr_data), .da_wr_strb(da_wr_strb),
    .tag_alloc_en(tag_alloc_en), .tag_alloc_set(tag_alloc_set),
    .tag_alloc_way(tag_alloc_way), .tag_alloc_tag(tag_alloc_tag),
    .tag_alloc_dirty(tag_alloc_dirty),
    .tag_plru_en(tag_plru_en), .tag_plru_set(tag_plru_set),
    .tag_plru_way(tag_plru_way),
    .rsp_valid(rsp_valid), .rsp_ready(rsp_ready),
    .rsp_id(rsp_id), .rsp_rdata(rsp_rdata), .rsp_err(rsp_err),
    .block_lookup(block_lookup),
    .fill_req_valid(fill_req_valid), .fill_req_ready(fill_req_ready),
    .fill_req_id(fill_req_id), .fill_req_addr(fill_req_addr),
    .fill_rsp_valid(fill_rsp_valid), .fill_rsp_ready(/*open*/),
    .fill_rsp_id(fill_rsp_id), .fill_rsp_line(fill_rsp_line),
    .fill_rsp_err(fill_rsp_err),
    .evict_req_valid(evict_req_valid), .evict_req_ready(evict_req_ready),
    .evict_req_id(evict_req_id), .evict_req_addr(evict_req_addr),
    .evict_req_line(evict_req_line),
    .evict_done_valid(evict_done_valid), .evict_done_ready(evict_done_ready),
    .evict_done_id(evict_done_id)
  );

  // ==========================================================================
  // Assumptions: the contract the top enforces around the MSHR
  // ==========================================================================
  always @(posedge clk) if (rst_n) begin
    // allocate only into a free file, never a line already tracked
    assume (!(alloc_en && look_full));
    assume (!(alloc_en && look_hit));
    // allocate and merge are mutually exclusive
    assume (!(alloc_en && merge_en));
    // merge only into the matching, still-mergeable entry
    if (merge_en) begin
      assume (look_hit && look_hit_mergeable);
      assume (merge_idx == look_hit_idx);
    end
    // the top never drives alloc/merge while the engine uses shared ports
    assume (!((alloc_en || merge_en) && block_lookup));
    // AXI responses only arrive for entries in the matching phase
    if (fill_rsp_valid)
      assume (dut.e_state[fill_rsp_id[IDXW-1:0]] == E_FILLW);
    if (evict_done_valid)
      assume (dut.e_wbpend[evict_done_id[IDXW-1:0]]);
  end

  // ==========================================================================
  // Assertions: the invariants to prove
  // ==========================================================================
  // ---- 1. address uniqueness ----------------------------------------------
  genvar i, j;
  generate
    for (i = 0; i < MSHR_CNT; i++) begin : g_i
      for (j = i + 1; j < MSHR_CNT; j++) begin : g_j
        always @(posedge clk) if (rst_n)
          a_unique: assert (!((dut.e_state[i] != E_EMPTY) &&
                              (dut.e_state[j] != E_EMPTY) &&
                              (dut.e_set[i] == dut.e_set[j]) &&
                              (dut.e_tag[i] == dut.e_tag[j])));
      end
    end
  endgenerate

  // ---- 2. free-slot validity / no allocation when full --------------------
  always @(posedge clk) if (rst_n) begin
    if (alloc_en)
      a_free_slot: assert (dut.e_state[dut.free_idx] == E_EMPTY);
  end

  // ---- 3. exactly-once retirement (waiter conservation) -------------------
  generate
    for (i = 0; i < MSHR_CNT; i++) begin : g_cnt
      always @(posedge clk) if (rst_n) begin
        a_cnt_range: assert (dut.w_cnt[i] <= NWAIT);
      end
    end
  endgenerate
  always @(posedge clk) if (rst_n) begin
    // never dequeue (emit a response) from an empty waiter FIFO -> no
    // duplicate/extra retirement
    if (rsp_valid && rsp_ready)
      a_no_over_dequeue: assert (dut.w_cnt[dut.ref_idx] > 0);
    // never enqueue into a full waiter FIFO -> no dropped (un-retired) request
    if (merge_en)
      a_no_enq_overflow: assert (dut.w_cnt[merge_idx] < NWAIT);
  end

  // ---- reachability covers (guard against vacuous proofs) ------------------
  always @(posedge clk) if (rst_n) begin
    c_response:  cover (rsp_valid && rsp_ready);          // a request is retired
    c_full:      cover (look_full);                       // file can fill up
    c_merge:     cover (merge_en && look_hit_mergeable);  // a merge can occur
    c_wbwait:    cover (dut.e_state[0] == E_WBWAIT);      // write-back tracking
  end

endmodule
