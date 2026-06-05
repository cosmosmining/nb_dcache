// ============================================================================
// nb_dcache_replayq.sv  -- replay queue (Stage 4)
// ----------------------------------------------------------------------------
// A small FIFO of requests that hit a structural hazard and must be retried:
//   * MSHRs full on a primary miss,
//   * every way of the target set already reserved by an MSHR ("set conflict
//     during refill"),
//   * the matching MSHR's waiter FIFO is full (cannot merge a secondary miss),
//   * the matching MSHR is past the mergeable window (being drained).
//
// Replay policy (see top): once anything is queued, the CPU port is closed and
// requests recirculate from the queue HEAD with head-of-line blocking, so the
// program order of the stalled stream is preserved. A head entry that still
// cannot proceed stays at the head; it makes progress when an in-flight fill
// frees an MSHR / unlocks a way, so there is no deadlock (AXI always responds).
// ============================================================================
`include "nb_dcache_pkg.sv"

module nb_dcache_replayq
  import nb_dcache_pkg::*;
#(
  parameter int unsigned DEPTH = 8
)(
  input  logic     clk,
  input  logic     rst_n,

  input  logic     push_en,
  input  cpu_req_t push_req,

  input  logic     pop_en,

  output logic     head_valid,
  output cpu_req_t head_req,
  output logic     full,
  output logic     empty
);

  localparam int unsigned PW = (DEPTH > 1) ? $clog2(DEPTH) : 1;

  cpu_req_t        mem [DEPTH];
  logic [PW-1:0]   head_q, tail_q;
  logic [PW:0]     cnt_q;

  assign empty      = (cnt_q == 0);
  assign full       = (cnt_q == (PW+1)'(DEPTH));
  assign head_valid = !empty;
  assign head_req   = mem[head_q];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      head_q <= '0; tail_q <= '0; cnt_q <= '0;
    end else begin
      if (push_en && !full) begin
        mem[tail_q] <= push_req;
        tail_q      <= (tail_q == (PW)'(DEPTH-1)) ? '0 : tail_q + 1'b1;
      end
      if (pop_en && !empty) begin
        head_q <= (head_q == (PW)'(DEPTH-1)) ? '0 : head_q + 1'b1;
      end
      case ({push_en && !full, pop_en && !empty})
        2'b10: cnt_q <= cnt_q + 1'b1;
        2'b01: cnt_q <= cnt_q - 1'b1;
        default: ; // 00 or simultaneous push+pop: count unchanged
      endcase
    end
  end

endmodule
