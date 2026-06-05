// ============================================================================
// nb_dcache_sva.sv  -- assertion checkers + bind statements (simulation)
// ----------------------------------------------------------------------------
// Bound non-intrusively into the RTL so the same DUT is checked by every
// cocotb test. Two checkers:
//   * nb_dcache_mshr_sva  (bound into nb_dcache_mshr): MSHR invariants
//   * nb_dcache_top_sva   (bound into nb_dcache_top) : AXI legality, retirement,
//                                                      pLRU/eviction safety
//
// Enable with Verilator's --assert (the cocotb Makefile does this).
// Formal proofs of the MSHR invariants live separately in formal/ (yosys/sby
// understands only a simple SVA subset).
// ============================================================================
`include "nb_dcache_pkg.sv"

// ----------------------------------------------------------------------------
// MSHR-file invariants
// ----------------------------------------------------------------------------
module nb_dcache_mshr_sva
  import nb_dcache_pkg::*;
(
  input logic         clk,
  input logic         rst_n,
  input mshr_estate_e e_state [MSHR_CNT],
  input set_t         e_set   [MSHR_CNT],
  input tag_t         e_tag   [MSHR_CNT],
  input logic         alloc_en
);
  // ---- no two MSHRs hold the same line address --------------------------
  for (genvar i = 0; i < MSHR_CNT; i++) begin : g_uniq_i
    for (genvar j = i + 1; j < MSHR_CNT; j++) begin : g_uniq_j
      a_mshr_unique: assert property (@(posedge clk) disable iff (!rst_n)
        !((e_state[i] != E_EMPTY) && (e_state[j] != E_EMPTY) &&
          (e_set[i] == e_set[j]) && (e_tag[i] == e_tag[j])))
        else $error("SVA: two MSHRs share a line (%0d,%0d)", i, j);
    end
  end

  // ---- no allocation when the file is full ------------------------------
  logic full;
  always_comb begin
    full = 1'b1;
    for (int k = 0; k < MSHR_CNT; k++)
      if (e_state[k] == E_EMPTY) full = 1'b0;
  end
  a_no_alloc_full: assert property (@(posedge clk) disable iff (!rst_n)
    !(alloc_en && full))
    else $error("SVA: allocate while MSHR file full");
endmodule

// ----------------------------------------------------------------------------
// Top-level: AXI legality, request retirement, eviction safety
// ----------------------------------------------------------------------------
module nb_dcache_top_sva
  import nb_dcache_pkg::*;
(
  input logic clk,
  input logic rst_n,
  // eviction safety
  input logic            do_alloc,
  input way_t            victim_way,
  input logic [WAYS-1:0] mshr_locked,
  // CPU port
  input logic            cpu_req_valid, cpu_req_ready,
  input logic [CPU_ID_W-1:0] cpu_req_id,
  input logic            cpu_rsp_valid, cpu_rsp_ready,
  input logic [CPU_ID_W-1:0] cpu_rsp_id,
  // AXI
  input logic m_arvalid, m_arready, input addr_t m_araddr,
  input axi_id_t m_arid, input logic [7:0] m_arlen,
  input logic m_rvalid, m_rready, input axi_id_t m_rid, input logic m_rlast,
  input logic m_awvalid, m_awready, input addr_t m_awaddr,
  input axi_id_t m_awid, input logic [7:0] m_awlen,
  input logic m_wvalid, m_wready, input data_t m_wdata,
  input strb_t m_wstrb, input logic m_wlast,
  input logic m_bvalid, m_bready, input axi_id_t m_bid
);
  // ---- pLRU never evicts a way reserved by an active MSHR ----------------
  a_evict_unlocked: assert property (@(posedge clk) disable iff (!rst_n)
    do_alloc |-> !mshr_locked[victim_way])
    else $error("SVA: evicting an MSHR-locked way");

  // ---- every accepted request retires exactly once ----------------------
  // pending[id] is set at accept, cleared at response. An accept may only use a
  // non-pending id (no id reuse / no double-allocate), and a response may only
  // fire for a pending id (no orphan / no double-retire).
  logic acc, rsp;
  assign acc = cpu_req_valid && cpu_req_ready;
  assign rsp = cpu_rsp_valid && cpu_rsp_ready;
  logic pending [1<<CPU_ID_W];
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) for (int k = 0; k < (1<<CPU_ID_W); k++) pending[k] <= 1'b0;
    else begin
      if (acc) pending[cpu_req_id] <= 1'b1;
      if (rsp) pending[cpu_rsp_id] <= 1'b0;
    end
  end
  a_accept_unique: assert property (@(posedge clk) disable iff (!rst_n)
    acc |-> !pending[cpu_req_id])
    else $error("SVA: request id %0d accepted while still outstanding", cpu_req_id);
  // A response is legal if its id was outstanding, OR it is being accepted this
  // very cycle (a single-cycle hit accepts and responds in one cycle).
  a_no_orphan_rsp: assert property (@(posedge clk) disable iff (!rst_n)
    rsp |-> (pending[cpu_rsp_id] || (acc && cpu_req_id == cpu_rsp_id)))
    else $error("SVA: response for non-outstanding id %0d", cpu_rsp_id);

  // ---- AXI: address/data stable while valid & not yet accepted -----------
  a_ar_stable: assert property (@(posedge clk) disable iff (!rst_n)
    (m_arvalid && !m_arready) |=>
      (m_arvalid && $stable(m_araddr) && $stable(m_arid) && $stable(m_arlen)));
  a_aw_stable: assert property (@(posedge clk) disable iff (!rst_n)
    (m_awvalid && !m_awready) |=>
      (m_awvalid && $stable(m_awaddr) && $stable(m_awid) && $stable(m_awlen)));
  a_w_stable: assert property (@(posedge clk) disable iff (!rst_n)
    (m_wvalid && !m_wready) |=>
      (m_wvalid && $stable(m_wdata) && $stable(m_wstrb) && $stable(m_wlast)));

  // ---- AXI: no orphan responses + bounded outstanding (ID bookkeeping) ---
  // The outstanding counters are *global* AR-vs-R / AW-vs-B accounting and are
  // intentionally NOT cleared by the DUT reset: a mid-burst reset abandons the
  // master's view, but the slave still completes every transaction it accepted,
  // so accounting stays balanced and "closes out" (returns to 0) once drained.
  // The bound tolerates one reset transient (old + new outstanding).
  logic [3:0] rd_out = '0, wr_out = '0;
  logic ar_fire, r_done, aw_fire, b_fire;
  assign ar_fire = m_arvalid && m_arready;
  assign r_done  = m_rvalid && m_rready && m_rlast;
  assign aw_fire = m_awvalid && m_awready;
  assign b_fire  = m_bvalid && m_bready;
  always_ff @(posedge clk) begin
    rd_out <= rd_out + (ar_fire ? 4'd1 : 4'd0) - (r_done ? 4'd1 : 4'd0);
    wr_out <= wr_out + (aw_fire ? 4'd1 : 4'd0) - (b_fire ? 4'd1 : 4'd0);
  end
  a_no_orphan_r: assert property (@(posedge clk)
    (m_rvalid && m_rready) |-> (rd_out != 0))
    else $error("SVA: R response with no outstanding read");
  a_no_orphan_b: assert property (@(posedge clk)
    b_fire |-> (wr_out != 0))
    else $error("SVA: B response with no outstanding write");
  a_rd_bounded: assert property (@(posedge clk) rd_out <= 4'(2*MSHR_CNT));
  a_wr_bounded: assert property (@(posedge clk) wr_out <= 4'(2*MSHR_CNT));
endmodule

// ----------------------------------------------------------------------------
// Bind statements
// ----------------------------------------------------------------------------
bind nb_dcache_mshr nb_dcache_mshr_sva u_mshr_sva (
  .clk(clk), .rst_n(rst_n),
  .e_state(e_state), .e_set(e_set), .e_tag(e_tag),
  .alloc_en(alloc_en)
);

bind nb_dcache_top nb_dcache_top_sva u_top_sva (
  .clk(clk), .rst_n(rst_n),
  .do_alloc(do_alloc), .victim_way(victim_way), .mshr_locked(mshr_locked),
  .cpu_req_valid(cpu_req_valid), .cpu_req_ready(cpu_req_ready),
  .cpu_req_id(cpu_req_id),
  .cpu_rsp_valid(cpu_rsp_valid), .cpu_rsp_ready(cpu_rsp_ready),
  .cpu_rsp_id(cpu_rsp_id),
  .m_arvalid(m_arvalid), .m_arready(m_arready), .m_araddr(m_araddr),
  .m_arid(m_arid), .m_arlen(m_arlen),
  .m_rvalid(m_rvalid), .m_rready(m_rready), .m_rid(m_rid), .m_rlast(m_rlast),
  .m_awvalid(m_awvalid), .m_awready(m_awready), .m_awaddr(m_awaddr),
  .m_awid(m_awid), .m_awlen(m_awlen),
  .m_wvalid(m_wvalid), .m_wready(m_wready), .m_wdata(m_wdata),
  .m_wstrb(m_wstrb), .m_wlast(m_wlast),
  .m_bvalid(m_bvalid), .m_bready(m_bready), .m_bid(m_bid)
);
