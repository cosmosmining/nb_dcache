// ============================================================================
// nb_dcache_maint.sv  -- cache maintenance engine (synchronous-read arrays)
// ----------------------------------------------------------------------------
// Whole-cache maintenance, run only when the cache is quiesced (no live MSHR,
// empty replay queue, lookup idle); the top gates new CPU traffic while `busy`.
//
//   req_flush = 0 : INVALIDATE ALL  -- drop every line (no write-back)
//   req_flush = 1 : FLUSH ALL       -- write back every dirty line, then
//                                      invalidate every line
//
// The tag/data arrays are synchronous-read and way-parallel, so the walk uses a
// SETUP phase (present walk_set, read_req=1) followed by a READ phase in which
// all ways of that set are available; ways are then iterated without re-reading,
// and only a new set needs another SETUP. A dirty line under flush is written
// back through the (idle) AXI evict port before being invalidated.
// ============================================================================
`include "nb_dcache_pkg.sv"

module nb_dcache_maint
  import nb_dcache_pkg::*;
(
  input  logic    clk,
  input  logic    rst_n,

  input  logic    req_valid,
  input  logic    req_flush,
  input  logic    quiesced,
  output logic    busy,
  output logic    done,

  // array walk (drives the muxed synchronous read in the top)
  output logic    read_req,     // present walk_set this cycle (rd_en)
  output set_t    walk_set,
  output way_t    walk_way,
  input  logic    cur_valid,    // valid one cycle after read_req, per walk_way
  input  logic    cur_dirty,
  input  tag_t    cur_tag,
  input  line_t   cur_line,

  output logic    inv_en,

  output logic    evict_valid,
  input  logic    evict_ready,
  output addr_t   evict_addr,
  output line_t   evict_line,
  output axi_id_t evict_id,
  input  logic    evict_done
);

  typedef enum logic [2:0] {
    M_IDLE, M_SETUP, M_READ, M_EVICT, M_EVWAIT, M_INV, M_NEXT, M_DONE
  } mstate_e;
  mstate_e mstate;

  set_t s_q;
  way_t w_q;
  logic flush_q;

  assign walk_set = s_q;
  assign walk_way = w_q;
  assign busy     = (mstate != M_IDLE);
  assign read_req = (mstate == M_SETUP);

  assign evict_valid = (mstate == M_EVICT);
  assign evict_addr  = {cur_tag, s_q, {OFFSET_BITS{1'b0}}};
  assign evict_line  = cur_line;
  assign evict_id    = '0;

  assign inv_en      = (mstate == M_INV);
  assign done        = (mstate == M_DONE);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mstate  <= M_IDLE;
      s_q     <= '0;
      w_q     <= '0;
      flush_q <= 1'b0;
    end else begin
      unique case (mstate)
        M_IDLE: if (req_valid && quiesced) begin
                  s_q     <= '0;
                  w_q     <= '0;
                  flush_q <= req_flush;
                  mstate  <= M_SETUP;
                end
        M_SETUP: mstate <= M_READ;          // walk_set presented; data next cycle
        M_READ: begin
          if (flush_q && cur_valid && cur_dirty) mstate <= M_EVICT;
          else if (cur_valid)                    mstate <= M_INV;
          else                                   mstate <= M_NEXT;
        end
        M_EVICT:  if (evict_ready) mstate <= M_EVWAIT;
        M_EVWAIT: if (evict_done)  mstate <= M_INV;
        M_INV:    mstate <= M_NEXT;
        M_NEXT: begin
          if (w_q != way_t'(WAYS-1)) begin
            w_q    <= w_q + 1'b1;
            mstate <= M_READ;               // same set still in the read latch
          end else if (s_q != set_t'(NUM_SETS-1)) begin
            w_q    <= '0;
            s_q    <= s_q + 1'b1;
            mstate <= M_SETUP;              // new set needs a fresh read
          end else begin
            mstate <= M_DONE;
          end
        end
        M_DONE: mstate <= M_IDLE;
        default: mstate <= M_IDLE;
      endcase
    end
  end

endmodule
