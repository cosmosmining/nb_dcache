// ============================================================================
// nb_dcache_axi.sv
// ----------------------------------------------------------------------------
// AXI4 master engine: turns the cache's line-granular fill/evict requests into
// burst transactions and reassembles responses.
//
//  Reads  (line fill) : one AR burst of BURST_BEATS beats per request, tagged
//                       with ARID = fill_req_id. R beats are demuxed by RID into
//                       per-ID line buffers, so bursts for different IDs may be
//                       interleaved / returned OUT OF ORDER on the R channel and
//                       still reassemble correctly. RLAST of an ID raises a
//                       per-ID "done" flag that the top drains via fill_rsp.
//
//  Writes (eviction)  : one AW + W burst per dirty victim. AXI4 has no WID, so
//                       the W data phase is serialized (one active stream at a
//                       time, AXI-legal), but B responses are tracked per BID and
//                       may complete OUT OF ORDER relative to other writes.
//
// The internal fill/evict ports are simple valid/ready. The top guarantees a
// read ID is not reused until its fill_rsp is consumed, so no ID alias can occur
// on the AR/R side.
// ============================================================================
`include "nb_dcache_pkg.sv"

module nb_dcache_axi
  import nb_dcache_pkg::*;
#(
  parameter int unsigned RD_IDS = MSHR_CNT,  // outstanding read IDs (0..RD_IDS-1)
  parameter int unsigned WR_IDS = MSHR_CNT   // outstanding write IDs (0..WR_IDS-1)
)(
  input  logic       clk,
  input  logic       rst_n,

  // ---- internal fill (read) request/response -------------------------------
  input  logic       fill_req_valid,
  output logic       fill_req_ready,
  input  axi_id_t    fill_req_id,
  input  addr_t      fill_req_addr,   // line aligned

  output logic       fill_rsp_valid,
  input  logic       fill_rsp_ready,
  output axi_id_t    fill_rsp_id,
  output line_t      fill_rsp_line,
  output logic       fill_rsp_err,    // any beat returned SLVERR/DECERR

  // ---- internal evict (write) request/completion ---------------------------
  input  logic       evict_req_valid,
  output logic       evict_req_ready,
  input  axi_id_t    evict_req_id,
  input  addr_t      evict_req_addr,  // line aligned
  input  line_t      evict_req_line,

  output logic       evict_done_valid,
  input  logic       evict_done_ready,
  output axi_id_t    evict_done_id,
  output logic       evict_done_err,  // write-back returned SLVERR/DECERR

  // ---- AXI4 master: read address -------------------------------------------
  output logic              m_arvalid,
  input  logic              m_arready,
  output axi_id_t           m_arid,
  output addr_t             m_araddr,
  output logic [7:0]        m_arlen,
  output logic [2:0]        m_arsize,
  output logic [1:0]        m_arburst,
  output logic [2:0]        m_arprot,
  output logic [3:0]        m_arcache,
  output logic              m_arlock,
  output logic [3:0]        m_arqos,
  // ---- AXI4 master: read data ----------------------------------------------
  input  logic              m_rvalid,
  output logic              m_rready,
  input  axi_id_t           m_rid,
  input  data_t             m_rdata,
  input  logic [1:0]        m_rresp,
  input  logic              m_rlast,
  // ---- AXI4 master: write address ------------------------------------------
  output logic              m_awvalid,
  input  logic              m_awready,
  output axi_id_t           m_awid,
  output addr_t             m_awaddr,
  output logic [7:0]        m_awlen,
  output logic [2:0]        m_awsize,
  output logic [1:0]        m_awburst,
  output logic [2:0]        m_awprot,
  output logic [3:0]        m_awcache,
  output logic              m_awlock,
  output logic [3:0]        m_awqos,
  // ---- AXI4 master: write data ---------------------------------------------
  output logic              m_wvalid,
  input  logic              m_wready,
  output data_t             m_wdata,
  output strb_t             m_wstrb,
  output logic              m_wlast,
  // ---- AXI4 master: write response -----------------------------------------
  input  logic              m_bvalid,
  output logic              m_bready,
  input  axi_id_t           m_bid,
  input  logic [1:0]        m_bresp
);

  localparam int unsigned RIDXW = (RD_IDS > 1) ? $clog2(RD_IDS) : 1;
  localparam int unsigned WIDXW = (WR_IDS > 1) ? $clog2(WR_IDS) : 1;

  // ==========================================================================
  // Read (fill) path
  // ==========================================================================
  // --- AR issue: 1-deep latch; serializes address issue but allows many
  //     data-phase bursts to be outstanding simultaneously on R. ------------
  logic    ar_pending_q;
  axi_id_t ar_id_q;
  addr_t   ar_addr_q;

  assign fill_req_ready = !ar_pending_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ar_pending_q <= 1'b0;
      ar_id_q      <= '0;
      ar_addr_q    <= '0;
    end else begin
      if (fill_req_valid && fill_req_ready) begin
        ar_pending_q <= 1'b1;
        ar_id_q      <= fill_req_id;
        ar_addr_q    <= fill_req_addr;
      end else if (m_arvalid && m_arready) begin
        ar_pending_q <= 1'b0;
      end
    end
  end

  assign m_arvalid = ar_pending_q;
  assign m_arid    = ar_id_q;
  assign m_araddr  = ar_addr_q;
  assign m_arlen   = 8'(AXLEN);
  assign m_arsize  = 3'($clog2(WORD_BYTES));
  assign m_arburst = 2'b01;    // INCR
  // sideband: data access, normal/secure; cacheable write-back read-allocate
  assign m_arprot  = 3'b000;
  assign m_arcache = 4'b1111;  // WB, read+write-allocate, modifiable, bufferable
  assign m_arlock  = 1'b0;
  assign m_arqos   = 4'b0000;

  // --- R reassembly: per-ID line buffer + beat counter + done flag ----------
  line_t                line_buf_q [RD_IDS];
  logic [WOFF_BITS-1:0] beat_cnt_q [RD_IDS];
  logic                 done_q     [RD_IDS];
  logic                 err_q      [RD_IDS]; // sticky error across the burst

  // We can always sink read data: there is dedicated storage per ID.
  assign m_rready = 1'b1;

  logic [RIDXW-1:0] rid_idx;
  assign rid_idx = m_rid[RIDXW-1:0];

  // Response arbiter: pick the lowest-index ID whose burst has completed.
  logic             rsp_hit;
  logic [RIDXW-1:0] rsp_idx;
  always_comb begin
    rsp_hit = 1'b0;
    rsp_idx = '0;
    for (int i = RD_IDS-1; i >= 0; i--)
      if (done_q[i]) begin
        rsp_hit = 1'b1;
        rsp_idx = RIDXW'(i);
      end
  end

  assign fill_rsp_valid = rsp_hit;
  assign fill_rsp_id    = axi_id_t'(rsp_idx);
  assign fill_rsp_line  = line_buf_q[rsp_idx];
  assign fill_rsp_err   = err_q[rsp_idx];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < RD_IDS; i++) begin
        beat_cnt_q[i] <= '0;
        done_q[i]     <= 1'b0;
        err_q[i]      <= 1'b0;
        line_buf_q[i] <= '0;
      end
    end else begin
      // accept a beat
      if (m_rvalid && m_rready) begin
        line_buf_q[rid_idx][beat_cnt_q[rid_idx]*DATA_WIDTH +: DATA_WIDTH] <= m_rdata;
        // RRESP[1] set => SLVERR/DECERR; make it sticky over the burst
        err_q[rid_idx] <= (beat_cnt_q[rid_idx] == '0)
                          ? m_rresp[1]
                          : (err_q[rid_idx] | m_rresp[1]);
        if (m_rlast) begin
          beat_cnt_q[rid_idx] <= '0;
          done_q[rid_idx]     <= 1'b1;
        end else begin
          beat_cnt_q[rid_idx] <= beat_cnt_q[rid_idx] + 1'b1;
        end
      end
      // drain a completed response
      if (fill_rsp_valid && fill_rsp_ready)
        done_q[rsp_idx] <= 1'b0;
    end
  end

  // ==========================================================================
  // Write (eviction) path
  // ==========================================================================
  // Serialized AW+W issue. State machine: IDLE -> AW -> WDATA. B responses are
  // tracked separately and may complete out of order across BIDs.
  typedef enum logic [1:0] {W_IDLE, W_ADDR, W_DATA} wstate_e;
  wstate_e              wstate_q;
  axi_id_t              w_id_q;
  addr_t                w_addr_q;
  line_t                w_line_q;
  logic [WOFF_BITS-1:0] w_beat_q;

  assign evict_req_ready = (wstate_q == W_IDLE);

  assign m_awvalid = (wstate_q == W_ADDR);
  assign m_awid    = w_id_q;
  assign m_awaddr  = w_addr_q;
  assign m_awlen   = 8'(AXLEN);
  assign m_awsize  = 3'($clog2(WORD_BYTES));
  assign m_awburst = 2'b01;    // INCR
  assign m_awprot  = 3'b000;
  assign m_awcache = 4'b1111;  // WB, write-allocate, modifiable, bufferable
  assign m_awlock  = 1'b0;
  assign m_awqos   = 4'b0000;

  assign m_wvalid  = (wstate_q == W_DATA);
  assign m_wdata   = w_line_q[w_beat_q*DATA_WIDTH +: DATA_WIDTH];
  assign m_wstrb   = '1; // full-line write-back: every byte valid
  assign m_wlast   = (wstate_q == W_DATA) && (w_beat_q == WOFF_BITS'(BURST_BEATS-1));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wstate_q <= W_IDLE;
      w_id_q   <= '0;
      w_addr_q <= '0;
      w_line_q <= '0;
      w_beat_q <= '0;
    end else begin
      unique case (wstate_q)
        W_IDLE: begin
          if (evict_req_valid) begin
            wstate_q <= W_ADDR;
            w_id_q   <= evict_req_id;
            w_addr_q <= evict_req_addr;
            w_line_q <= evict_req_line;
            w_beat_q <= '0;
          end
        end
        W_ADDR: begin
          if (m_awready) wstate_q <= W_DATA;
        end
        W_DATA: begin
          if (m_wready) begin
            if (w_beat_q == WOFF_BITS'(BURST_BEATS-1)) begin
              wstate_q <= W_IDLE;
              w_beat_q <= '0;
            end else begin
              w_beat_q <= w_beat_q + 1'b1;
            end
          end
        end
        default: wstate_q <= W_IDLE;
      endcase
    end
  end

  // --- B response tracking: per-ID done flags, drained via evict_done --------
  logic             bdone_q [WR_IDS];
  logic             berr_q  [WR_IDS];
  logic [WIDXW-1:0] bid_idx;
  assign bid_idx = m_bid[WIDXW-1:0];

  assign m_bready = 1'b1; // dedicated per-ID completion storage

  logic             bsp_hit;
  logic [WIDXW-1:0] bsp_idx;
  always_comb begin
    bsp_hit = 1'b0;
    bsp_idx = '0;
    for (int i = WR_IDS-1; i >= 0; i--)
      if (bdone_q[i]) begin
        bsp_hit = 1'b1;
        bsp_idx = WIDXW'(i);
      end
  end

  assign evict_done_valid = bsp_hit;
  assign evict_done_id    = axi_id_t'(bsp_idx);
  assign evict_done_err   = berr_q[bsp_idx];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < WR_IDS; i++) begin
        bdone_q[i] <= 1'b0;
        berr_q[i]  <= 1'b0;
      end
    end else begin
      if (m_bvalid && m_bready) begin
        bdone_q[bid_idx] <= 1'b1;
        berr_q[bid_idx]  <= m_bresp[1]; // SLVERR/DECERR
      end
      if (evict_done_valid && evict_done_ready) bdone_q[bsp_idx] <= 1'b0;
    end
  end

  // Tie off the low (informational) RESP bits; bit[1] is consumed above.
  logic _unused;
  assign _unused = &{1'b0, m_rresp[0], m_bresp[0]};

endmodule
