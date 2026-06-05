// ============================================================================
// nb_dcache_mshr.sv  -- MSHR file / miss engine
// ----------------------------------------------------------------------------
// Owns the entire miss path so the top can keep the hit path simple:
//
//   * Lookup (comb)      : reports line hit / free slot / full / per-set locked
//                          ways, so the top can decide hit/miss/merge/replay in
//                          the same cycle as the tag compare.
//   * Allocate           : primary miss -> new entry, first waiter, optional
//                          dirty-victim eviction.
//   * Merge              : secondary miss to an in-flight line -> append a waiter
//                          (per-word sub-entries):
//                             - STORE merges its bytes into wr_data/wr_strb of
//                               its word (write merging into an in-flight MSHR)
//                               AND parks a store-ack waiter.
//                             - LOAD captures the *currently merged* store bytes
//                               for its word at enqueue time (store-to-load
//                               forwarding during the fill window) and parks a
//                               load waiter. Because the snapshot is taken at
//                               enqueue, a load that arrived BEFORE a store sees
//                               memory, and one that arrived AFTER sees the
//                               store -- program order is preserved per byte.
//   * AXI phases         : per-entry EVICT -> FILL -> FILLW, allowing multiple
//                          outstanding fills/evicts (miss-under-miss). ARID/AWID
//                          = entry index, so out-of-order R/B across IDs route
//                          back to the right entry.
//   * Refill controller  : one entry at a time writes the refilled+merged line
//                          into the data array, allocates the tag, then DRAINs
//                          its waiters to the CPU response port. While it uses
//                          those shared ports it raises block_lookup so the
//                          top's hit path stalls for those few cycles; during
//                          the long AXI latency the hit path runs freely
//                          (hit-under-miss).
//
// Invariant (checked by SVA): a new entry is only allocated on an mshr MISS, so
// no two entries ever hold the same line address.
// ============================================================================
`include "nb_dcache_pkg.sv"

module nb_dcache_mshr
  import nb_dcache_pkg::*;
#(
  parameter int unsigned NWAIT = 8   // max parked requests (waiters) per entry
)(
  input  logic clk,
  input  logic rst_n,

  // ---- lookup (combinational) ----------------------------------------------
  input  set_t  look_set,
  input  tag_t  look_tag,
  output logic  look_hit,        // some entry holds {look_tag,look_set}
  output logic  look_hit_mergeable,
  output logic [$clog2(MSHR_CNT)-1:0] look_hit_idx,
  output logic  look_full,       // no free entry
  output logic [WAYS-1:0] look_locked_ways, // ways reserved by entries in look_set
  // A primary miss whose line equals an in-flight eviction's address must wait:
  // issuing its fill (read) before the write-back (write) completes would be a
  // WAR hazard at memory. Replay until the eviction's B response lands.
  output logic  look_wb_conflict,
  output logic [$clog2(MSHR_CNT+1)-1:0] dbg_occupancy, // live entries (coverage)

  // ---- allocate a new entry (primary miss) ---------------------------------
  input  logic  alloc_en,
  input  set_t  alloc_set,
  input  tag_t  alloc_tag,
  input  way_t  alloc_way,
  input  logic  alloc_need_evict,
  input  tag_t  alloc_vtag,      // victim tag (for the eviction address)
  input  line_t alloc_vline,     // victim line data (read from data array by top)
  // first waiter:
  input  cpu_id_t alloc_w_id,
  input  logic    alloc_w_isld,
  input  woff_t   alloc_w_word,
  input  data_t   alloc_w_data,
  input  strb_t   alloc_w_strb,

  // ---- merge into an existing entry (secondary miss) ------------------------
  input  logic  merge_en,
  input  logic [$clog2(MSHR_CNT)-1:0] merge_idx,
  input  cpu_id_t merge_w_id,
  input  logic    merge_w_isld,
  input  woff_t   merge_w_word,
  input  data_t   merge_w_data,
  input  strb_t   merge_w_strb,

  // ---- shared resource: data array write (refill) --------------------------
  output logic      da_wr_en,
  output set_t      da_wr_set,
  output way_t      da_wr_way,
  output line_t     da_wr_data,
  output linestrb_t da_wr_strb,

  // ---- shared resource: tag array allocate ---------------------------------
  output logic  tag_alloc_en,
  output set_t  tag_alloc_set,
  output way_t  tag_alloc_way,
  output tag_t  tag_alloc_tag,
  output logic  tag_alloc_dirty,
  output logic  tag_plru_en,
  output set_t  tag_plru_set,
  output way_t  tag_plru_way,

  // ---- shared resource: CPU response port ----------------------------------
  output logic    rsp_valid,
  input  logic    rsp_ready,
  output cpu_id_t rsp_id,
  output data_t   rsp_rdata,
  output logic    rsp_err,

  // ---- hit path stall (this engine is using shared ports this cycle) --------
  output logic  block_lookup,

  // ---- AXI engine: fill (read) ---------------------------------------------
  output logic    fill_req_valid,
  input  logic    fill_req_ready,
  output axi_id_t fill_req_id,
  output addr_t   fill_req_addr,
  input  logic    fill_rsp_valid,
  output logic    fill_rsp_ready,
  input  axi_id_t fill_rsp_id,
  input  line_t   fill_rsp_line,
  input  logic    fill_rsp_err,

  // ---- AXI engine: evict (write) -------------------------------------------
  output logic    evict_req_valid,
  input  logic    evict_req_ready,
  output axi_id_t evict_req_id,
  output addr_t   evict_req_addr,
  output line_t   evict_req_line,
  input  logic    evict_done_valid,
  output logic    evict_done_ready,
  input  axi_id_t evict_done_id
);

  localparam int unsigned IDXW = (MSHR_CNT > 1) ? $clog2(MSHR_CNT) : 1;

  // entry lifecycle state type comes from the package (mshr_estate_e)

  // ---- per-entry state ------------------------------------------------------
  mshr_estate_e  e_state [MSHR_CNT];
  tag_t     e_tag   [MSHR_CNT];
  set_t     e_set   [MSHR_CNT];
  way_t     e_way   [MSHR_CNT];
  tag_t     e_vtag  [MSHR_CNT];
  line_t    e_vline [MSHR_CNT];
  line_t    e_mem   [MSHR_CNT];                 // captured fill line
  logic     e_err   [MSHR_CNT];                 // fill returned a bus error
  logic     e_wbpend[MSHR_CNT];                 // eviction write-back outstanding (until B)
  data_t    e_wdata [MSHR_CNT][WORDS_PER_LN];   // merged store data per word
  strb_t    e_wstrb [MSHR_CNT][WORDS_PER_LN];   // merged store strobes per word

  // ---- per-entry waiter FIFO ------------------------------------------------
  cpu_id_t  w_id    [MSHR_CNT][NWAIT];
  woff_t    w_word  [MSHR_CNT][NWAIT];
  data_t    w_fdata [MSHR_CNT][NWAIT];
  strb_t    w_fstrb [MSHR_CNT][NWAIT];
  localparam int unsigned WPTRW = $clog2(NWAIT);
  logic [WPTRW:0]   w_cnt  [MSHR_CNT];
  logic [WPTRW-1:0] w_head [MSHR_CNT];
  logic [WPTRW-1:0] w_tail [MSHR_CNT];

  // ==========================================================================
  // Combinational lookup
  // ==========================================================================
  always_comb begin
    look_hit           = 1'b0;
    look_hit_mergeable = 1'b0;
    look_hit_idx       = '0;
    look_full          = 1'b1;
    look_locked_ways   = '0;
    look_wb_conflict   = 1'b0;
    for (int i = 0; i < MSHR_CNT; i++) begin
      if (e_state[i] != E_EMPTY) begin
        if (e_set[i] == look_set) look_locked_ways[e_way[i]] = 1'b1;
        // pending write-back of the *evicted* line at {e_vtag,e_set}
        if (e_wbpend[i] && e_set[i] == look_set && e_vtag[i] == look_tag)
          look_wb_conflict = 1'b1;
        if (e_set[i] == look_set && e_tag[i] == look_tag) begin
          look_hit     = 1'b1;
          look_hit_idx = IDXW'(i);
          // mergeable while still gathering (not yet handed to refill ctrl) and
          // the waiter FIFO has room.
          look_hit_mergeable =
            (e_state[i] == E_EVICT || e_state[i] == E_FILL ||
             e_state[i] == E_FILLW) && (w_cnt[i] < (WPTRW+1)'(NWAIT));
        end
      end else begin
        look_full = 1'b0;
      end
    end
  end

  // occupancy (number of live entries) for functional coverage
  always_comb begin
    dbg_occupancy = '0;
    for (int i = 0; i < MSHR_CNT; i++)
      if (e_state[i] != E_EMPTY)
        dbg_occupancy = dbg_occupancy + 1'b1;
  end

  // first free entry index
  logic [IDXW-1:0] free_idx;
  always_comb begin
    free_idx = '0;
    for (int i = MSHR_CNT-1; i >= 0; i--)
      if (e_state[i] == E_EMPTY) free_idx = IDXW'(i);
  end

  // ==========================================================================
  // AXI request arbitration (lowest index wins)
  // ==========================================================================
  logic [IDXW-1:0] evict_idx; logic evict_any;
  logic [IDXW-1:0] fill_idx;  logic fill_any;
  always_comb begin
    evict_any = 1'b0; evict_idx = '0;
    fill_any  = 1'b0; fill_idx  = '0;
    for (int i = MSHR_CNT-1; i >= 0; i--) begin
      if (e_state[i] == E_EVICT) begin evict_any = 1'b1; evict_idx = IDXW'(i); end
      if (e_state[i] == E_FILL)  begin fill_any  = 1'b1; fill_idx  = IDXW'(i); end
    end
  end

  assign evict_req_valid = evict_any;
  assign evict_req_id    = axi_id_t'(evict_idx);
  assign evict_req_addr  = {e_vtag[evict_idx], e_set[evict_idx], {OFFSET_BITS{1'b0}}};
  assign evict_req_line  = e_vline[evict_idx];
  assign evict_done_ready = 1'b1;

  assign fill_req_valid  = fill_any;
  assign fill_req_id     = axi_id_t'(fill_idx);
  assign fill_req_addr   = {e_tag[fill_idx], e_set[fill_idx], {OFFSET_BITS{1'b0}}};
  assign fill_rsp_ready  = 1'b1;

  logic [IDXW-1:0] rsp_fill_idx;
  assign rsp_fill_idx = fill_rsp_id[IDXW-1:0];

  // ==========================================================================
  // Refill controller (one entry uses data/tag/response ports at a time)
  // ==========================================================================
  typedef enum logic [1:0] { RC_IDLE, RC_WRITE, RC_DRAIN } rcstate_e;
  rcstate_e         rc_state;
  logic [IDXW-1:0]  ref_idx;

  // pick a ready-to-refill entry
  logic             refreq_any;
  logic [IDXW-1:0] refreq_idx;
  always_comb begin
    refreq_any = 1'b0; refreq_idx = '0;
    for (int i = MSHR_CNT-1; i >= 0; i--)
      if (e_state[i] == E_REFILL_REQ) begin refreq_any = 1'b1; refreq_idx = IDXW'(i); end
  end

  assign block_lookup = (rc_state == RC_WRITE) || (rc_state == RC_DRAIN);

  // assembled refill line: merged store bytes over memory bytes
  line_t refill_line;
  logic  refill_dirty;
  always_comb begin
    refill_line  = e_mem[ref_idx];
    refill_dirty = 1'b0;
    for (int wd = 0; wd < WORDS_PER_LN; wd++) begin
      for (int b = 0; b < WORD_BYTES; b++)
        if (e_wstrb[ref_idx][wd][b])
          refill_line[(wd*WORD_BYTES + b)*8 +: 8] = e_wdata[ref_idx][wd][b*8 +: 8];
      if (|e_wstrb[ref_idx][wd]) refill_dirty = 1'b1;
    end
  end

  // ---- data array write & tag allocate during RC_WRITE ----------------------
  // On a bus error the line is NOT installed: skip the data write, the tag
  // allocate and the pLRU touch so the (invalidated) victim way stays free and a
  // subsequent access re-fetches. Waiters are still drained, each with rsp_err.
  logic refill_ok;
  assign refill_ok  = (rc_state == RC_WRITE) && !e_err[ref_idx];

  assign da_wr_en   = refill_ok;
  assign da_wr_set  = e_set[ref_idx];
  assign da_wr_way  = e_way[ref_idx];
  assign da_wr_data = refill_line;
  assign da_wr_strb = '1;

  assign tag_alloc_en    = refill_ok;
  assign tag_alloc_set   = e_set[ref_idx];
  assign tag_alloc_way   = e_way[ref_idx];
  assign tag_alloc_tag   = e_tag[ref_idx];
  assign tag_alloc_dirty = refill_dirty;
  assign tag_plru_en     = refill_ok;
  assign tag_plru_set    = e_set[ref_idx];
  assign tag_plru_way    = e_way[ref_idx];

  // ---- drain: compute the head waiter's response ----------------------------
  logic [WPTRW-1:0] dh;                  // drain head pointer
  assign dh = w_head[ref_idx];

  data_t drain_rdata;
  always_comb begin
    drain_rdata = e_mem[ref_idx][w_word[ref_idx][dh]*DATA_WIDTH +: DATA_WIDTH];
    for (int b = 0; b < WORD_BYTES; b++)
      if (w_fstrb[ref_idx][dh][b])
        drain_rdata[b*8 +: 8] = w_fdata[ref_idx][dh][b*8 +: 8];
  end

  assign rsp_valid = (rc_state == RC_DRAIN) && (w_cnt[ref_idx] != 0);
  assign rsp_id    = w_id[ref_idx][dh];
  assign rsp_rdata = drain_rdata;
  assign rsp_err   = e_err[ref_idx];   // bus error reported to every waiter

  // ==========================================================================
  // Sequential state
  // ==========================================================================
  // Local helpers to push a waiter / merge a store, applied inside always_ff.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rc_state <= RC_IDLE;
      ref_idx  <= '0;
      for (int i = 0; i < MSHR_CNT; i++) begin
        e_state[i] <= E_EMPTY;
        e_tag[i] <= '0; e_set[i] <= '0; e_way[i] <= '0;
        e_vtag[i] <= '0; e_vline[i] <= '0; e_mem[i] <= '0;
        e_err[i] <= 1'b0; e_wbpend[i] <= 1'b0;
        w_cnt[i] <= '0; w_head[i] <= '0; w_tail[i] <= '0;
        for (int wd = 0; wd < WORDS_PER_LN; wd++) begin
          e_wdata[i][wd] <= '0; e_wstrb[i][wd] <= '0;
        end
      end
    end else begin
      // ---- per-entry AXI phase transitions ---------------------------------
      for (int i = 0; i < MSHR_CNT; i++) begin
        case (e_state[i])
          E_EVICT: if (evict_any && evict_idx == IDXW'(i) && evict_req_ready)
                     e_state[i] <= E_FILL;
          E_FILL:  if (fill_any && fill_idx == IDXW'(i) && fill_req_ready)
                     e_state[i] <= E_FILLW;
          E_FILLW: if (fill_rsp_valid && rsp_fill_idx == IDXW'(i)) begin
                     e_mem[i]   <= fill_rsp_line;
                     e_err[i]   <= fill_rsp_err;
                     e_state[i] <= E_REFILL_REQ;
                   end
          default: ; // E_EMPTY / E_REFILL_REQ handled by ctrl/alloc
        endcase
      end

      // ---- refill controller ----------------------------------------------
      case (rc_state)
        RC_IDLE: if (refreq_any) begin
                   ref_idx  <= refreq_idx;
                   rc_state <= RC_WRITE;
                 end
        RC_WRITE: rc_state <= RC_DRAIN;   // line write + tag alloc issued combinationally
        RC_DRAIN: begin
          if (w_cnt[ref_idx] == 0) begin
            // all waiters served. If the eviction's B is still outstanding, hold
            // the entry (locked, tracked) in E_WBWAIT; else free it.
            e_state[ref_idx] <=
              (e_wbpend[ref_idx] &&
               !(evict_done_valid && evict_done_id[IDXW-1:0] == ref_idx))
              ? E_WBWAIT : E_EMPTY;
            // clear merged store state for reuse
            for (int wd = 0; wd < WORDS_PER_LN; wd++) e_wstrb[ref_idx][wd] <= '0;
            w_head[ref_idx] <= '0; w_tail[ref_idx] <= '0;
            rc_state <= RC_IDLE;
          end else if (rsp_valid && rsp_ready) begin
            w_head[ref_idx] <= w_head[ref_idx] + 1'b1;
            w_cnt[ref_idx]  <= w_cnt[ref_idx] - 1'b1;
          end
        end
        default: rc_state <= RC_IDLE;
      endcase

      // ---- allocate (primary miss) -----------------------------------------
      if (alloc_en) begin
        e_state[free_idx] <= alloc_need_evict ? E_EVICT : E_FILL;
        e_tag[free_idx]   <= alloc_tag;
        e_set[free_idx]   <= alloc_set;
        e_way[free_idx]   <= alloc_way;
        e_vtag[free_idx]  <= alloc_vtag;
        e_vline[free_idx] <= alloc_vline;
        e_wbpend[free_idx] <= alloc_need_evict; // write-back tracked until B
        // reset merged store state
        for (int wd = 0; wd < WORDS_PER_LN; wd++) begin
          e_wdata[free_idx][wd] <= '0;
          e_wstrb[free_idx][wd] <= '0;
        end
        // install first waiter (and merge a store into its word)
        w_id[free_idx][0]    <= alloc_w_id;
        w_word[free_idx][0]  <= alloc_w_word;
        if (alloc_w_isld) begin
          w_fdata[free_idx][0] <= '0; // no prior store -> reads pure memory
          w_fstrb[free_idx][0] <= '0;
        end else begin
          w_fdata[free_idx][0] <= '0;
          w_fstrb[free_idx][0] <= '0;
          // merge store bytes
          for (int b = 0; b < WORD_BYTES; b++)
            if (alloc_w_strb[b]) begin
              e_wdata[free_idx][alloc_w_word][b*8 +: 8] <= alloc_w_data[b*8 +: 8];
              e_wstrb[free_idx][alloc_w_word][b]        <= 1'b1;
            end
        end
        w_head[free_idx] <= '0;
        w_tail[free_idx] <= WPTRW'(1);
        w_cnt[free_idx]  <= 1;
      end

      // ---- merge (secondary miss) ------------------------------------------
      if (merge_en) begin
        // append waiter at tail
        w_id[merge_idx][w_tail[merge_idx]]   <= merge_w_id;
        w_word[merge_idx][w_tail[merge_idx]] <= merge_w_word;
        if (merge_w_isld) begin
          // snapshot currently-merged store bytes for this word (forwarding)
          w_fdata[merge_idx][w_tail[merge_idx]] <= e_wdata[merge_idx][merge_w_word];
          w_fstrb[merge_idx][w_tail[merge_idx]] <= e_wstrb[merge_idx][merge_w_word];
        end else begin
          w_fdata[merge_idx][w_tail[merge_idx]] <= '0;
          w_fstrb[merge_idx][w_tail[merge_idx]] <= '0;
          // merge store bytes (last writer per byte wins)
          for (int b = 0; b < WORD_BYTES; b++)
            if (merge_w_strb[b]) begin
              e_wdata[merge_idx][merge_w_word][b*8 +: 8] <= merge_w_data[b*8 +: 8];
              e_wstrb[merge_idx][merge_w_word][b]        <= 1'b1;
            end
        end
        w_tail[merge_idx] <= w_tail[merge_idx] + 1'b1;
        w_cnt[merge_idx]  <= w_cnt[merge_idx] + 1'b1;
      end

      // ---- eviction write-back completion (B response) ---------------------
      if (evict_done_valid) begin
        e_wbpend[evict_done_id[IDXW-1:0]] <= 1'b0;
        if (e_state[evict_done_id[IDXW-1:0]] == E_WBWAIT)
          e_state[evict_done_id[IDXW-1:0]] <= E_EMPTY;
      end
    end
  end

  // is_load is not needed downstream (store responses carry don't-care rdata).
  logic _unused_mshr;
  assign _unused_mshr = &{1'b0, alloc_w_isld, merge_w_isld, fill_rsp_id};

endmodule
