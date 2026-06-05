// ============================================================================
// nb_dcache_top.sv  -- non-blocking L1 data cache top
// ----------------------------------------------------------------------------
// Integrates the hit path with the MSHR miss engine, replay queue and AXI
// engine. A single-cycle lookup pipeline serves one request per cycle from
// either the replay queue (head-of-line, priority) or the CPU port.
//
// Per-request decision (when the engine is not using shared ports):
//   tag hit                         -> service immediately (load/store), respond
//   tag miss, MSHR hit, mergeable   -> merge as a secondary miss (no new MSHR)
//   tag miss, no MSHR, way+slot free -> allocate a new MSHR (primary miss),
//                                       invalidate the victim, kick off
//                                       eviction (if dirty) + fill
//   otherwise (structural hazard)   -> push to the replay queue
//
// Shared-port arbitration: the miss engine raises block_lookup for the few
// cycles it writes the refilled line / drains responses; the hit path stalls
// then. During the long AXI latency block_lookup is low, so hits proceed
// (hit-under-miss) and further misses allocate other MSHRs (miss-under-miss).
//
// Hazards handled explicitly (see also docs/README hazard table):
//   * store-to-load forwarding in the fill window  -> MSHR per-word load snapshot
//   * write merging into an in-flight MSHR         -> MSHR store merge
//   * eviction targeting a set with a pending miss -> victim masked to unlocked
//                                                     ways; victim invalidated at
//                                                     alloc so the doomed line
//                                                     cannot be hit mid-fill
//   * re-access of a line whose write-back is in flight -> look_wb_conflict replay
// ============================================================================
`include "nb_dcache_pkg.sv"

module nb_dcache_top
  import nb_dcache_pkg::*;
#(
  parameter int unsigned MSHR_NWAIT  = 8,
  parameter int unsigned REPLAY_DEPTH = 8
)(
  input  logic     clk,
  input  logic     rst_n,

  // ---- CPU side (flat, tagged valid/ready) ---------------------------------
  input  logic                cpu_req_valid,
  output logic                cpu_req_ready,
  input  logic [CPU_ID_W-1:0] cpu_req_id,
  input  addr_t               cpu_req_addr,
  input  logic                cpu_req_we,
  input  data_t               cpu_req_wdata,
  input  strb_t               cpu_req_wstrb,

  output logic                cpu_rsp_valid,
  input  logic                cpu_rsp_ready,
  output logic [CPU_ID_W-1:0] cpu_rsp_id,
  output data_t               cpu_rsp_rdata,
  output logic                cpu_rsp_err,

  // ---- cache maintenance ----------------------------------------------------
  input  logic                maint_req_valid,  // pulse to start (when idle)
  input  logic                maint_req_flush,  // 1=flush(WB)+inv, 0=invalidate
  output logic                maint_busy,
  output logic                maint_done,       // 1-cycle pulse on completion

  // ---- AXI4 master ----------------------------------------------------------
  output logic              m_arvalid, input logic m_arready,
  output axi_id_t           m_arid,    output addr_t m_araddr,
  output logic [7:0]        m_arlen,   output logic [2:0] m_arsize,
  output logic [1:0]        m_arburst,
  output logic [2:0]        m_arprot,  output logic [3:0] m_arcache,
  output logic              m_arlock,  output logic [3:0] m_arqos,
  input  logic              m_rvalid,  output logic m_rready,
  input  axi_id_t           m_rid,     input  data_t m_rdata,
  input  logic [1:0]        m_rresp,   input  logic m_rlast,
  output logic              m_awvalid, input logic m_awready,
  output axi_id_t           m_awid,    output addr_t m_awaddr,
  output logic [7:0]        m_awlen,   output logic [2:0] m_awsize,
  output logic [1:0]        m_awburst,
  output logic [2:0]        m_awprot,  output logic [3:0] m_awcache,
  output logic              m_awlock,  output logic [3:0] m_awqos,
  output logic              m_wvalid,  input logic m_wready,
  output data_t             m_wdata,   output strb_t m_wstrb,
  output logic              m_wlast,
  input  logic              m_bvalid,  output logic m_bready,
  input  axi_id_t           m_bid,     input  logic [1:0] m_bresp,

  // ---- debug / functional-coverage bus (synthesizable, observation only) ----
  output logic [$clog2(MSHR_CNT+1)-1:0] dbg_mshr_occupancy,
  output logic                          dbg_merge,        // a secondary miss merged
  output logic                          dbg_replay,       // a request entered replayq
  output logic                          dbg_evict_dirty,  // alloc with dirty write-back
  output logic                          dbg_evict_clean,  // alloc replacing clean line
  output logic [3:0]                    dbg_hazard,       // {mergewin,wbc,novictim,full}

  // ---- performance counters (free-running; sample/clear externally) ---------
  output logic [31:0]                   perf_hits,
  output logic [31:0]                   perf_misses,
  output logic [31:0]                   perf_replays,
  output logic [31:0]                   perf_writebacks,
  output logic [31:0]                   perf_bus_errors
);

  localparam int unsigned IDXW = (MSHR_CNT > 1) ? $clog2(MSHR_CNT) : 1;

  // ==========================================================================
  // Pipeline input: replay queue head (priority) or CPU request
  // ==========================================================================
  cpu_req_t cpu_req;
  always_comb begin
    cpu_req.id    = cpu_req_id;
    cpu_req.addr  = cpu_req_addr;
    cpu_req.we    = cpu_req_we;
    cpu_req.wdata = cpu_req_wdata;
    cpu_req.wstrb = cpu_req_wstrb;
  end

  logic     rq_head_valid; cpu_req_t rq_head_req;
  logic     rq_full, rq_empty;
  logic     rq_push_en, rq_pop_en;

  logic     from_replay;
  cpu_req_t in_req;
  logic     in_valid;
  assign from_replay = rq_head_valid;
  assign in_req      = from_replay ? rq_head_req : cpu_req;
  assign in_valid    = from_replay ? 1'b1 : cpu_req_valid;

  set_t  in_set;  tag_t in_tag;  woff_t in_woff;
  assign in_set  = addr_set (in_req.addr);
  assign in_tag  = addr_tag (in_req.addr);
  assign in_woff = addr_woff(in_req.addr);

  // ==========================================================================
  // Tag array
  // ==========================================================================
  tag_t                 ta_rd_tag   [WAYS];
  logic                 ta_rd_valid [WAYS];
  logic                 ta_rd_dirty [WAYS];
  logic [PLRU_BITS-1:0] ta_rd_plru;

  logic alloc_en;  set_t alloc_set; way_t alloc_way; tag_t alloc_tag; logic alloc_dirty;
  logic sd_en;     set_t sd_set;    way_t sd_way;
  logic inv_en;    set_t inv_set;   way_t inv_way;
  logic plru_en;   set_t plru_set;  way_t plru_way;

  // maintenance engine taps (declared here, driven below)
  logic  maint_busy_w, maint_inv_en;
  set_t  maint_walk_set;
  way_t  maint_walk_way;
  set_t  rd_set_mux;   // tag/data read address (lookup or maintenance walk)
  assign rd_set_mux = maint_busy_w ? maint_walk_set : in_set;
  // invalidate port: maintenance walk vs. primary-miss victim
  logic  ta_inv_en;  set_t ta_inv_set;  way_t ta_inv_way;
  assign ta_inv_en  = maint_busy_w ? maint_inv_en   : inv_en;
  assign ta_inv_set = maint_busy_w ? maint_walk_set : inv_set;
  assign ta_inv_way = maint_busy_w ? maint_walk_way : inv_way;

  nb_dcache_tag_array u_tags (
    .clk(clk), .rst_n(rst_n),
    .rd_set(rd_set_mux), .rd_tag(ta_rd_tag), .rd_valid(ta_rd_valid),
    .rd_dirty(ta_rd_dirty), .rd_plru(ta_rd_plru),
    .alloc_en(alloc_en), .alloc_set(alloc_set), .alloc_way(alloc_way),
    .alloc_tag(alloc_tag), .alloc_dirty(alloc_dirty),
    .setdirty_en(sd_en), .setdirty_set(sd_set), .setdirty_way(sd_way),
    .inv_en(ta_inv_en), .inv_set(ta_inv_set), .inv_way(ta_inv_way),
    .plru_en(plru_en), .plru_set(plru_set), .plru_way(plru_way)
  );

  // ==========================================================================
  // Data array
  // ==========================================================================
  way_t      da_rd_way;
  line_t     da_rd_line;
  logic      da_wr_en;   set_t da_wr_set; way_t da_wr_way;
  line_t     da_wr_data; linestrb_t da_wr_strb;

  way_t da_rd_way_mux;
  assign da_rd_way_mux = maint_busy_w ? maint_walk_way : da_rd_way;

  nb_dcache_data_array u_data (
    .clk(clk),
    .rd_set(rd_set_mux), .rd_way(da_rd_way_mux), .rd_line(da_rd_line),
    .wr_en(da_wr_en), .wr_set(da_wr_set), .wr_way(da_wr_way),
    .wr_data(da_wr_data), .wr_strb(da_wr_strb)
  );

  // ==========================================================================
  // Tag compare + victim selection
  // ==========================================================================
  logic hit;  way_t hit_way;
  always_comb begin
    hit = 1'b0; hit_way = '0;
    for (int w = 0; w < WAYS; w++)
      if (ta_rd_valid[w] && ta_rd_tag[w] == in_tag) begin
        hit = 1'b1; hit_way = way_t'(w);
      end
  end

  // MSHR lookup wires
  logic            mshr_look_hit, mshr_look_merge, mshr_look_full, mshr_look_wbc;
  logic [IDXW-1:0] mshr_look_idx;
  logic [WAYS-1:0] mshr_locked;

  // choose an unlocked victim, preferring the pLRU victim
  way_t victim_pref, victim_way;  logic have_victim;
  assign victim_pref = plru_victim(ta_rd_plru);
  always_comb begin
    have_victim = 1'b0; victim_way = '0;
    if (!mshr_locked[victim_pref]) begin
      have_victim = 1'b1; victim_way = victim_pref;
    end else begin
      for (int w = WAYS-1; w >= 0; w--)
        if (!mshr_locked[w]) begin have_victim = 1'b1; victim_way = way_t'(w); end
    end
  end

  logic victim_dirty;
  assign victim_dirty = ta_rd_valid[victim_way] && ta_rd_dirty[victim_way];

  // data array read selects hit way (hit) or victim way (miss, to capture line)
  assign da_rd_way = hit ? hit_way : victim_way;

  // ==========================================================================
  // Miss engine (MSHR file)
  // ==========================================================================
  // mshr -> shared resource drivers
  logic      m_da_wr_en;   set_t m_da_wr_set; way_t m_da_wr_way;
  line_t     m_da_wr_data; linestrb_t m_da_wr_strb;
  logic      m_tag_alloc_en; set_t m_tag_alloc_set; way_t m_tag_alloc_way;
  tag_t      m_tag_alloc_tag; logic m_tag_alloc_dirty;
  logic      m_tag_plru_en; set_t m_tag_plru_set; way_t m_tag_plru_way;
  logic      m_rsp_valid; cpu_id_t m_rsp_id; data_t m_rsp_rdata; logic m_rsp_ready;
  logic      block_lookup;

  // mshr <-> AXI
  logic    fill_req_valid, fill_req_ready; axi_id_t fill_req_id; addr_t fill_req_addr;
  logic    fill_rsp_valid, fill_rsp_ready; axi_id_t fill_rsp_id; line_t fill_rsp_line;
  logic    fill_rsp_err;
  logic    evict_req_valid, evict_req_ready; axi_id_t evict_req_id;
  addr_t   evict_req_addr; line_t evict_req_line;
  logic    evict_done_valid, evict_done_ready; axi_id_t evict_done_id;
  logic    evict_done_err, m_rsp_err;
  // MSHR's evict request (muxed with the maintenance engine's below)
  logic    mshr_evict_valid; axi_id_t mshr_evict_id;
  addr_t   mshr_evict_addr;  line_t mshr_evict_line;
  // maintenance engine's evict request
  logic    maint_evict_valid; axi_id_t maint_evict_id;
  addr_t   maint_evict_addr;  line_t maint_evict_line;

  // control: per-request actions (declared here, driven below)
  logic do_hit, do_merge, do_alloc, do_hazard;
  logic alloc_fire, merge_fire;

  nb_dcache_mshr #(.NWAIT(MSHR_NWAIT)) u_mshr (
    .clk(clk), .rst_n(rst_n),
    .look_set(in_set), .look_tag(in_tag),
    .look_hit(mshr_look_hit), .look_hit_mergeable(mshr_look_merge),
    .look_hit_idx(mshr_look_idx), .look_full(mshr_look_full),
    .look_locked_ways(mshr_locked), .look_wb_conflict(mshr_look_wbc),
    .dbg_occupancy(dbg_mshr_occupancy),

    .alloc_en(alloc_fire), .alloc_set(in_set), .alloc_tag(in_tag),
    .alloc_way(victim_way), .alloc_need_evict(victim_dirty),
    .alloc_vtag(ta_rd_tag[victim_way]), .alloc_vline(da_rd_line),
    .alloc_w_id(in_req.id), .alloc_w_isld(~in_req.we), .alloc_w_word(in_woff),
    .alloc_w_data(in_req.wdata), .alloc_w_strb(in_req.wstrb),

    .merge_en(merge_fire), .merge_idx(mshr_look_idx),
    .merge_w_id(in_req.id), .merge_w_isld(~in_req.we), .merge_w_word(in_woff),
    .merge_w_data(in_req.wdata), .merge_w_strb(in_req.wstrb),

    .da_wr_en(m_da_wr_en), .da_wr_set(m_da_wr_set), .da_wr_way(m_da_wr_way),
    .da_wr_data(m_da_wr_data), .da_wr_strb(m_da_wr_strb),
    .tag_alloc_en(m_tag_alloc_en), .tag_alloc_set(m_tag_alloc_set),
    .tag_alloc_way(m_tag_alloc_way), .tag_alloc_tag(m_tag_alloc_tag),
    .tag_alloc_dirty(m_tag_alloc_dirty),
    .tag_plru_en(m_tag_plru_en), .tag_plru_set(m_tag_plru_set),
    .tag_plru_way(m_tag_plru_way),
    .rsp_valid(m_rsp_valid), .rsp_ready(m_rsp_ready),
    .rsp_id(m_rsp_id), .rsp_rdata(m_rsp_rdata), .rsp_err(m_rsp_err),
    .block_lookup(block_lookup),

    .fill_req_valid(fill_req_valid), .fill_req_ready(fill_req_ready),
    .fill_req_id(fill_req_id), .fill_req_addr(fill_req_addr),
    .fill_rsp_valid(fill_rsp_valid), .fill_rsp_ready(fill_rsp_ready),
    .fill_rsp_id(fill_rsp_id), .fill_rsp_line(fill_rsp_line),
    .fill_rsp_err(fill_rsp_err),
    .evict_req_valid(mshr_evict_valid), .evict_req_ready(evict_req_ready),
    .evict_req_id(mshr_evict_id), .evict_req_addr(mshr_evict_addr),
    .evict_req_line(mshr_evict_line),
    .evict_done_valid(evict_done_valid), .evict_done_ready(evict_done_ready),
    .evict_done_id(evict_done_id)
  );

  // evict port arbitration: the maintenance engine runs only when the cache is
  // quiesced (no live MSHR), so a simple mux suffices.
  assign evict_req_valid = maint_busy_w ? maint_evict_valid : mshr_evict_valid;
  assign evict_req_id    = maint_busy_w ? maint_evict_id    : mshr_evict_id;
  assign evict_req_addr  = maint_busy_w ? maint_evict_addr  : mshr_evict_addr;
  assign evict_req_line  = maint_busy_w ? maint_evict_line  : mshr_evict_line;

  // ==========================================================================
  // Maintenance engine
  // ==========================================================================
  logic quiesced;
  assign quiesced = (dbg_mshr_occupancy == '0) && rq_empty;

  nb_dcache_maint u_maint (
    .clk(clk), .rst_n(rst_n),
    .req_valid(maint_req_valid), .req_flush(maint_req_flush),
    .quiesced(quiesced), .busy(maint_busy_w), .done(maint_done),
    .walk_set(maint_walk_set), .walk_way(maint_walk_way),
    .cur_valid(ta_rd_valid[maint_walk_way]),
    .cur_dirty(ta_rd_dirty[maint_walk_way]),
    .cur_tag(ta_rd_tag[maint_walk_way]),
    .cur_line(da_rd_line),
    .inv_en(maint_inv_en),
    .evict_valid(maint_evict_valid), .evict_ready(evict_req_ready),
    .evict_addr(maint_evict_addr), .evict_line(maint_evict_line),
    .evict_id(maint_evict_id), .evict_done(evict_done_valid)
  );
  assign maint_busy = maint_busy_w;

  // ==========================================================================
  // Replay queue
  // ==========================================================================
  nb_dcache_replayq #(.DEPTH(REPLAY_DEPTH)) u_rq (
    .clk(clk), .rst_n(rst_n),
    .push_en(rq_push_en), .push_req(in_req),
    .pop_en(rq_pop_en),
    .head_valid(rq_head_valid), .head_req(rq_head_req),
    .full(rq_full), .empty(rq_empty)
  );

  // ==========================================================================
  // AXI engine
  // ==========================================================================
  nb_dcache_axi u_axi (
    .clk(clk), .rst_n(rst_n),
    .fill_req_valid(fill_req_valid), .fill_req_ready(fill_req_ready),
    .fill_req_id(fill_req_id), .fill_req_addr(fill_req_addr),
    .fill_rsp_valid(fill_rsp_valid), .fill_rsp_ready(fill_rsp_ready),
    .fill_rsp_id(fill_rsp_id), .fill_rsp_line(fill_rsp_line),
    .fill_rsp_err(fill_rsp_err),
    .evict_req_valid(evict_req_valid), .evict_req_ready(evict_req_ready),
    .evict_req_id(evict_req_id), .evict_req_addr(evict_req_addr),
    .evict_req_line(evict_req_line),
    .evict_done_valid(evict_done_valid), .evict_done_ready(evict_done_ready),
    .evict_done_id(evict_done_id), .evict_done_err(evict_done_err),
    .m_arvalid(m_arvalid), .m_arready(m_arready), .m_arid(m_arid),
    .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arsize(m_arsize), .m_arburst(m_arburst),
    .m_arprot(m_arprot), .m_arcache(m_arcache), .m_arlock(m_arlock), .m_arqos(m_arqos),
    .m_rvalid(m_rvalid), .m_rready(m_rready), .m_rid(m_rid), .m_rdata(m_rdata),
    .m_rresp(m_rresp), .m_rlast(m_rlast),
    .m_awvalid(m_awvalid), .m_awready(m_awready), .m_awid(m_awid),
    .m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awsize(m_awsize), .m_awburst(m_awburst),
    .m_awprot(m_awprot), .m_awcache(m_awcache), .m_awlock(m_awlock), .m_awqos(m_awqos),
    .m_wvalid(m_wvalid), .m_wready(m_wready), .m_wdata(m_wdata), .m_wstrb(m_wstrb),
    .m_wlast(m_wlast),
    .m_bvalid(m_bvalid), .m_bready(m_bready), .m_bid(m_bid), .m_bresp(m_bresp)
  );

  // ==========================================================================
  // Lookup decision
  // ==========================================================================
  logic process_req;
  assign process_req = in_valid && !block_lookup && !maint_busy_w;

  // hit-path response handshake (only when not blocked)
  logic hit_rsp_ready, hit_commit;
  assign hit_rsp_ready = !block_lookup && cpu_rsp_ready;

  always_comb begin
    do_hit    = process_req && hit;
    do_merge  = process_req && !hit && mshr_look_hit && mshr_look_merge;
    do_alloc  = process_req && !hit && !mshr_look_hit &&
                have_victim && !mshr_look_full && !mshr_look_wbc;
    do_hazard = process_req && !hit && !do_merge && !do_alloc;
  end

  assign hit_commit = do_hit && hit_rsp_ready;

  // commit conditions: alloc/merge never need the response port (they retire
  // later via drain), so they commit unconditionally once chosen.
  assign alloc_fire = do_alloc;
  assign merge_fire = do_merge;

  // ---- replay queue push/pop -----------------------------------------------
  always_comb begin
    rq_pop_en  = 1'b0;
    rq_push_en = 1'b0;
    if (from_replay) begin
      // head retires (or merges/allocs) -> pop; hazard -> leave at head
      rq_pop_en = hit_commit || do_merge || do_alloc;
    end else begin
      // CPU-sourced hazard is absorbed into the replay queue
      rq_push_en = do_hazard && !rq_full;
    end
  end

  // CPU port accepts only when the replay queue is empty and the request
  // actually makes progress this cycle.
  assign cpu_req_ready = !from_replay &&
                         (hit_commit || do_merge || do_alloc ||
                          (do_hazard && !rq_full));

  // ==========================================================================
  // Shared-resource muxing (miss engine has priority during block_lookup)
  // ==========================================================================
  // ---- data array write ----------------------------------------------------
  // hit store word placement
  line_t     st_line; linestrb_t st_strb;
  always_comb begin
    st_line = '0; st_strb = '0;
    st_line[in_woff*DATA_WIDTH +: DATA_WIDTH] = in_req.wdata;
    st_strb[in_woff*WORD_BYTES +: WORD_BYTES] = in_req.wstrb;
  end

  always_comb begin
    if (block_lookup) begin
      da_wr_en   = m_da_wr_en;
      da_wr_set  = m_da_wr_set;
      da_wr_way  = m_da_wr_way;
      da_wr_data = m_da_wr_data;
      da_wr_strb = m_da_wr_strb;
    end else begin
      da_wr_en   = do_hit && in_req.we && hit_rsp_ready;
      da_wr_set  = in_set;
      da_wr_way  = hit_way;
      da_wr_data = st_line;
      da_wr_strb = st_strb;
    end
  end

  // ---- tag array update ----------------------------------------------------
  always_comb begin
    // allocate (refill) only from the miss engine
    alloc_en    = block_lookup && m_tag_alloc_en;
    alloc_set   = m_tag_alloc_set;
    alloc_way   = m_tag_alloc_way;
    alloc_tag   = m_tag_alloc_tag;
    alloc_dirty = m_tag_alloc_dirty;

    // invalidate the victim at primary-miss allocation
    inv_en  = do_alloc;
    inv_set = in_set;
    inv_way = victim_way;

    // pLRU: refill touch (engine) or hit touch (lookup)
    if (block_lookup) begin
      plru_en  = m_tag_plru_en;
      plru_set = m_tag_plru_set;
      plru_way = m_tag_plru_way;
    end else begin
      plru_en  = hit_commit;
      plru_set = in_set;
      plru_way = hit_way;
    end

    // set-dirty on a committed store hit
    sd_en  = !block_lookup && do_hit && in_req.we && hit_rsp_ready;
    sd_set = in_set;
    sd_way = hit_way;
  end

  // ---- CPU response port ---------------------------------------------------
  data_t hit_rdata;
  assign hit_rdata = da_rd_line[in_woff*DATA_WIDTH +: DATA_WIDTH];

  assign cpu_rsp_valid = block_lookup ? m_rsp_valid : do_hit;
  assign cpu_rsp_id    = block_lookup ? m_rsp_id    : in_req.id;
  assign cpu_rsp_rdata = block_lookup ? m_rsp_rdata : hit_rdata;
  // hits never error; a miss reports the fill's bus error to its waiters
  assign cpu_rsp_err   = block_lookup ? m_rsp_err   : 1'b0;
  assign m_rsp_ready   = block_lookup && cpu_rsp_ready;

  // ---- functional-coverage event taps --------------------------------------
  assign dbg_merge       = merge_fire;
  assign dbg_replay      = rq_push_en;
  assign dbg_evict_dirty = do_alloc && victim_dirty;
  assign dbg_evict_clean = do_alloc && !victim_dirty && ta_rd_valid[victim_way];
  assign dbg_hazard[0]   = do_hazard && !mshr_look_hit && mshr_look_full;
  assign dbg_hazard[1]   = do_hazard && !mshr_look_hit && !have_victim;
  assign dbg_hazard[2]   = do_hazard && !mshr_look_hit && mshr_look_wbc;
  assign dbg_hazard[3]   = do_hazard && mshr_look_hit && !mshr_look_merge;

  // ---- performance counters ------------------------------------------------
  logic bus_err_event;
  assign bus_err_event = (fill_rsp_valid && fill_rsp_ready && fill_rsp_err) ||
                         (evict_done_valid && evict_done_ready && evict_done_err);
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      perf_hits       <= '0;
      perf_misses     <= '0;
      perf_replays    <= '0;
      perf_writebacks <= '0;
      perf_bus_errors <= '0;
    end else begin
      if (hit_commit)              perf_hits       <= perf_hits + 1'b1;
      if (alloc_fire || merge_fire)perf_misses     <= perf_misses + 1'b1;
      if (rq_push_en)              perf_replays    <= perf_replays + 1'b1;
      if (dbg_evict_dirty)         perf_writebacks <= perf_writebacks + 1'b1;
      if (bus_err_event)           perf_bus_errors <= perf_bus_errors + 1'b1;
    end
  end

  // tie-offs
  logic _unused_top;
  assign _unused_top = &{1'b0, rq_empty, fill_req_id};

endmodule
