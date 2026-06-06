// ============================================================================
// nb_dcache_top.sv  -- non-blocking L1 data cache top (SRAM / pipelined)
// ----------------------------------------------------------------------------
// The tag and data arrays are now SYNCHRONOUS-READ (SRAM-mappable), so lookup is
// a two-phase, non-pipelined sequence per request:
//
//   L_ADDR   : present the request's set to the (way-parallel) tag & data
//              arrays (rd_en=1) and latch the request into the s2 pipeline reg.
//   L_DECIDE : the registered tag/data reads are valid; do the tag compare,
//              victim/MSHR decision and commit (respond / store / alloc / merge /
//              replay). Always returns to L_ADDR; the request is only *accepted*
//              (cpu_req_ready / replay pop) on a successful commit, so a stall
//              (response back-pressure, block_lookup, full replay queue) simply
//              re-reads and retries -- the decision is never made on stale data.
//
// Because reads happen in L_ADDR and writes (store hit / refill) in L_DECIDE /
// during block_lookup, a read never collides with a write to the same set in the
// same cycle (no bypass needed). Throughput is one request per two cycles; the
// next step toward a 1/cycle pipeline is forwarding across the two phases.
//
// Miss handling, hazards and AXI behavior are unchanged from the single-cycle
// version (see the MSHR engine and the README hazard table).
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
  input  logic                maint_req_valid,  // hold high until busy
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
  output logic                          dbg_merge,
  output logic                          dbg_replay,
  output logic                          dbg_evict_dirty,
  output logic                          dbg_evict_clean,
  output logic [3:0]                    dbg_hazard,

  // ---- performance counters (free-running) ---------------------------------
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

  set_t  in_set;
  assign in_set  = addr_set (in_req.addr);

  // ==========================================================================
  // Two-phase lookup FSM
  // ==========================================================================
  typedef enum logic {L_ADDR, L_DECIDE} lstate_e;
  lstate_e  lstate;
  cpu_req_t s2_req;          // request under decision (read in L_ADDR)
  logic     s2_from_replay;

  set_t  s2_set;  tag_t s2_tag;  woff_t s2_woff;
  assign s2_set  = addr_set (s2_req.addr);
  assign s2_tag  = addr_tag (s2_req.addr);
  assign s2_woff = addr_woff(s2_req.addr);

  // maintenance taps (driven below)
  logic  maint_busy_w, maint_inv_en, maint_read_req;
  set_t  maint_walk_set;
  way_t  maint_walk_way;

  logic block_lookup;        // miss engine using shared ports (from MSHR)

  // start a new lookup read this cycle?
  logic lookup_start;
  assign lookup_start = (lstate == L_ADDR) && in_valid &&
                        !block_lookup && !maint_busy_w && !maint_req_valid;

  // shared array read address / enable (lookup address-phase or maintenance walk)
  logic  arr_rd_en;  set_t arr_rd_set;
  assign arr_rd_en  = maint_busy_w ? maint_read_req : lookup_start;
  assign arr_rd_set = maint_busy_w ? maint_walk_set : in_set;

  // ==========================================================================
  // Tag array (synchronous read)
  // ==========================================================================
  tag_t                 ta_rd_tag   [WAYS];
  logic                 ta_rd_valid [WAYS];
  logic                 ta_rd_dirty [WAYS];
  logic [PLRU_BITS-1:0] ta_rd_plru;

  logic alloc_en;  set_t alloc_set; way_t alloc_way; tag_t alloc_tag; logic alloc_dirty;
  logic sd_en;     set_t sd_set;    way_t sd_way;
  logic inv_en;    set_t inv_set;   way_t inv_way;
  logic plru_en;   set_t plru_set;  way_t plru_way;

  // invalidate port: maintenance walk vs. primary-miss victim
  logic  ta_inv_en;  set_t ta_inv_set;  way_t ta_inv_way;
  assign ta_inv_en  = maint_busy_w ? maint_inv_en   : inv_en;
  assign ta_inv_set = maint_busy_w ? maint_walk_set : inv_set;
  assign ta_inv_way = maint_busy_w ? maint_walk_way : inv_way;

  nb_dcache_tag_array u_tags (
    .clk(clk), .rst_n(rst_n),
    .rd_en(arr_rd_en), .rd_set(arr_rd_set),
    .rd_tag(ta_rd_tag), .rd_valid(ta_rd_valid),
    .rd_dirty(ta_rd_dirty), .rd_plru(ta_rd_plru),
    .alloc_en(alloc_en), .alloc_set(alloc_set), .alloc_way(alloc_way),
    .alloc_tag(alloc_tag), .alloc_dirty(alloc_dirty),
    .setdirty_en(sd_en), .setdirty_set(sd_set), .setdirty_way(sd_way),
    .inv_en(ta_inv_en), .inv_set(ta_inv_set), .inv_way(ta_inv_way),
    .plru_en(plru_en), .plru_set(plru_set), .plru_way(plru_way)
  );

  // ==========================================================================
  // Data array (synchronous, way-parallel read)
  // ==========================================================================
  line_t     da_rd_line [WAYS];
  logic      da_wr_en;   set_t da_wr_set; way_t da_wr_way;
  line_t     da_wr_data; linestrb_t da_wr_strb;

  nb_dcache_data_array u_data (
    .clk(clk),
    .rd_en(arr_rd_en), .rd_set(arr_rd_set), .rd_line(da_rd_line),
    .wr_en(da_wr_en), .wr_set(da_wr_set), .wr_way(da_wr_way),
    .wr_data(da_wr_data), .wr_strb(da_wr_strb)
  );

  // ==========================================================================
  // Tag compare + victim selection (uses s2 + registered reads, in L_DECIDE)
  // ==========================================================================
  logic hit;  way_t hit_way;
  always_comb begin
    hit = 1'b0; hit_way = '0;
    for (int w = 0; w < WAYS; w++)
      if (ta_rd_valid[w] && ta_rd_tag[w] == s2_tag) begin
        hit = 1'b1; hit_way = way_t'(w);
      end
  end

  logic            mshr_look_hit, mshr_look_merge, mshr_look_full, mshr_look_wbc;
  logic [IDXW-1:0] mshr_look_idx;
  logic [WAYS-1:0] mshr_locked;

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

  // ==========================================================================
  // Miss engine (MSHR file)
  // ==========================================================================
  logic      m_da_wr_en;   set_t m_da_wr_set; way_t m_da_wr_way;
  line_t     m_da_wr_data; linestrb_t m_da_wr_strb;
  logic      m_tag_alloc_en; set_t m_tag_alloc_set; way_t m_tag_alloc_way;
  tag_t      m_tag_alloc_tag; logic m_tag_alloc_dirty;
  logic      m_tag_plru_en; set_t m_tag_plru_set; way_t m_tag_plru_way;
  logic      m_rsp_valid; cpu_id_t m_rsp_id; data_t m_rsp_rdata; logic m_rsp_ready;

  logic    fill_req_valid, fill_req_ready; axi_id_t fill_req_id; addr_t fill_req_addr;
  logic    fill_rsp_valid, fill_rsp_ready; axi_id_t fill_rsp_id; line_t fill_rsp_line;
  logic    fill_rsp_err;
  logic    evict_req_valid, evict_req_ready; axi_id_t evict_req_id;
  addr_t   evict_req_addr; line_t evict_req_line;
  logic    evict_done_valid, evict_done_ready; axi_id_t evict_done_id;
  logic    evict_done_err, m_rsp_err;
  logic    mshr_evict_valid; axi_id_t mshr_evict_id;
  addr_t   mshr_evict_addr;  line_t mshr_evict_line;
  logic    maint_evict_valid; axi_id_t maint_evict_id;
  addr_t   maint_evict_addr;  line_t maint_evict_line;

  logic do_hit, do_merge, do_alloc, do_hazard;
  logic alloc_fire, merge_fire;

  nb_dcache_mshr #(.NWAIT(MSHR_NWAIT)) u_mshr (
    .clk(clk), .rst_n(rst_n),
    .look_set(s2_set), .look_tag(s2_tag),
    .look_hit(mshr_look_hit), .look_hit_mergeable(mshr_look_merge),
    .look_hit_idx(mshr_look_idx), .look_full(mshr_look_full),
    .look_locked_ways(mshr_locked), .look_wb_conflict(mshr_look_wbc),
    .dbg_occupancy(dbg_mshr_occupancy),

    .alloc_en(alloc_fire), .alloc_set(s2_set), .alloc_tag(s2_tag),
    .alloc_way(victim_way), .alloc_need_evict(victim_dirty),
    .alloc_vtag(ta_rd_tag[victim_way]), .alloc_vline(da_rd_line[victim_way]),
    .alloc_w_id(s2_req.id), .alloc_w_isld(~s2_req.we), .alloc_w_word(s2_woff),
    .alloc_w_data(s2_req.wdata), .alloc_w_strb(s2_req.wstrb),

    .merge_en(merge_fire), .merge_idx(mshr_look_idx),
    .merge_w_id(s2_req.id), .merge_w_isld(~s2_req.we), .merge_w_word(s2_woff),
    .merge_w_data(s2_req.wdata), .merge_w_strb(s2_req.wstrb),

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

  // evict port arbitration (maintenance runs only when quiesced)
  assign evict_req_valid = maint_busy_w ? maint_evict_valid : mshr_evict_valid;
  assign evict_req_id    = maint_busy_w ? maint_evict_id    : mshr_evict_id;
  assign evict_req_addr  = maint_busy_w ? maint_evict_addr  : mshr_evict_addr;
  assign evict_req_line  = maint_busy_w ? maint_evict_line  : mshr_evict_line;

  // ==========================================================================
  // Maintenance engine
  // ==========================================================================
  logic quiesced;
  assign quiesced = (dbg_mshr_occupancy == '0) && rq_empty && (lstate == L_ADDR);

  nb_dcache_maint u_maint (
    .clk(clk), .rst_n(rst_n),
    .req_valid(maint_req_valid), .req_flush(maint_req_flush),
    .quiesced(quiesced), .busy(maint_busy_w), .done(maint_done),
    .read_req(maint_read_req),
    .walk_set(maint_walk_set), .walk_way(maint_walk_way),
    .cur_valid(ta_rd_valid[maint_walk_way]),
    .cur_dirty(ta_rd_dirty[maint_walk_way]),
    .cur_tag(ta_rd_tag[maint_walk_way]),
    .cur_line(da_rd_line[maint_walk_way]),
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
    .push_en(rq_push_en), .push_req(s2_req),
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
  // Decision (only in L_DECIDE, when the miss engine isn't using the ports)
  // ==========================================================================
  logic process_req;
  assign process_req = (lstate == L_DECIDE) && !block_lookup;

  logic hit_rsp_ready, hit_commit;
  assign hit_rsp_ready = cpu_rsp_ready;   // (block_lookup already excluded)

  always_comb begin
    do_hit    = process_req && hit;
    do_merge  = process_req && !hit && mshr_look_hit && mshr_look_merge;
    do_alloc  = process_req && !hit && !mshr_look_hit &&
                have_victim && !mshr_look_full && !mshr_look_wbc;
    do_hazard = process_req && !hit && !do_merge && !do_alloc;
  end

  assign hit_commit = do_hit && hit_rsp_ready;
  assign alloc_fire = do_alloc;
  assign merge_fire = do_merge;

  // a request is "consumed" (input accepted / head popped) on a successful commit
  logic commit;
  assign commit = hit_commit || do_merge || do_alloc || (do_hazard && !rq_full);

  always_comb begin
    rq_pop_en  = 1'b0;
    rq_push_en = 1'b0;
    if (s2_from_replay) begin
      rq_pop_en = hit_commit || do_merge || do_alloc;  // hazard -> leave at head
    end else begin
      rq_push_en = do_hazard && !rq_full;              // cpu hazard -> enqueue
    end
  end

  // CPU port accepts (in L_DECIDE) only for a cpu-sourced request that commits
  assign cpu_req_ready = (lstate == L_DECIDE) && !s2_from_replay && commit;

  // ---- FSM sequencing -------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lstate         <= L_ADDR;
      s2_req         <= '0;
      s2_from_replay <= 1'b0;
    end else begin
      unique case (lstate)
        L_ADDR: if (lookup_start) begin
                  s2_req         <= in_req;
                  s2_from_replay <= from_replay;
                  lstate         <= L_DECIDE;
                end
        L_DECIDE: lstate <= L_ADDR;   // always return; commit gated above
        default:  lstate <= L_ADDR;
      endcase
    end
  end

  // ==========================================================================
  // Shared-resource muxing (miss engine has priority during block_lookup)
  // ==========================================================================
  line_t     st_line; linestrb_t st_strb;
  always_comb begin
    st_line = '0; st_strb = '0;
    st_line[s2_woff*DATA_WIDTH +: DATA_WIDTH] = s2_req.wdata;
    st_strb[s2_woff*WORD_BYTES +: WORD_BYTES] = s2_req.wstrb;
  end

  // ---- data array write ----------------------------------------------------
  always_comb begin
    if (block_lookup) begin
      da_wr_en   = m_da_wr_en;
      da_wr_set  = m_da_wr_set;
      da_wr_way  = m_da_wr_way;
      da_wr_data = m_da_wr_data;
      da_wr_strb = m_da_wr_strb;
    end else begin
      da_wr_en   = do_hit && s2_req.we && hit_rsp_ready;
      da_wr_set  = s2_set;
      da_wr_way  = hit_way;
      da_wr_data = st_line;
      da_wr_strb = st_strb;
    end
  end

  // ---- tag array update ----------------------------------------------------
  always_comb begin
    alloc_en    = block_lookup && m_tag_alloc_en;
    alloc_set   = m_tag_alloc_set;
    alloc_way   = m_tag_alloc_way;
    alloc_tag   = m_tag_alloc_tag;
    alloc_dirty = m_tag_alloc_dirty;

    inv_en  = do_alloc;          // invalidate the victim at primary-miss alloc
    inv_set = s2_set;
    inv_way = victim_way;

    if (block_lookup) begin
      plru_en  = m_tag_plru_en;
      plru_set = m_tag_plru_set;
      plru_way = m_tag_plru_way;
    end else begin
      plru_en  = hit_commit;
      plru_set = s2_set;
      plru_way = hit_way;
    end

    sd_en  = !block_lookup && do_hit && s2_req.we && hit_rsp_ready;
    sd_set = s2_set;
    sd_way = hit_way;
  end

  // ---- CPU response port ---------------------------------------------------
  data_t hit_rdata;
  assign hit_rdata = da_rd_line[hit_way][s2_woff*DATA_WIDTH +: DATA_WIDTH];

  assign cpu_rsp_valid = block_lookup ? m_rsp_valid : do_hit;
  assign cpu_rsp_id    = block_lookup ? m_rsp_id    : s2_req.id;
  assign cpu_rsp_rdata = block_lookup ? m_rsp_rdata : hit_rdata;
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
      if (hit_commit)               perf_hits       <= perf_hits + 1'b1;
      if (alloc_fire || merge_fire) perf_misses     <= perf_misses + 1'b1;
      if (rq_push_en)               perf_replays    <= perf_replays + 1'b1;
      if (dbg_evict_dirty)          perf_writebacks <= perf_writebacks + 1'b1;
      if (bus_err_event)            perf_bus_errors <= perf_bus_errors + 1'b1;
    end
  end

  // tie-offs
  logic _unused_top;
  assign _unused_top = &{1'b0, rq_empty, fill_req_id};

endmodule
