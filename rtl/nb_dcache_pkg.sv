// ============================================================================
// nb_dcache_pkg.sv
// ----------------------------------------------------------------------------
// Shared parameters and type definitions for the non-blocking L1 data cache.
//
// The package is written for the *final* non-blocking architecture so that the
// module boundaries (tag array, data array, MSHR file, replay queue, AXI
// engine, top) stay stable as the design is grown incrementally:
//
//   Stage 1: blocking write-back / write-allocate cache
//   Stage 2: + MSHRs (hit-under-miss)
//   Stage 3: + miss-under-miss with secondary-miss merging
//   Stage 4: + replay queue and structural-hazard handling
//
// All widths are derived from the three knobs CACHE_SIZE, LINE_BYTES, WAYS so
// the geometry can be re-parameterized without touching the RTL body.
// ============================================================================
`ifndef NB_DCACHE_PKG_SV
`define NB_DCACHE_PKG_SV

package nb_dcache_pkg;

  // --------------------------------------------------------------------------
  // Geometry knobs
  // --------------------------------------------------------------------------
  parameter int unsigned CACHE_SIZE  = 16 * 1024; // bytes, default 16 KiB
  parameter int unsigned LINE_BYTES  = 64;        // bytes per cache line
  parameter int unsigned WAYS        = 4;         // set associativity

  parameter int unsigned ADDR_WIDTH  = 32;        // CPU/AXI address width
  parameter int unsigned DATA_WIDTH  = 64;        // CPU & AXI data beat width
  parameter int unsigned CPU_ID_W    = 4;         // CPU request tag width
  parameter int unsigned AXI_ID_W    = 4;         // AXI transaction id width
  parameter int unsigned MSHR_CNT    = 4;         // number of MSHR entries

  // --------------------------------------------------------------------------
  // Derived geometry
  // --------------------------------------------------------------------------
  parameter int unsigned LINE_BITS    = LINE_BYTES * 8;             // 512
  parameter int unsigned WORD_BYTES   = DATA_WIDTH / 8;            // 8
  parameter int unsigned WORDS_PER_LN = LINE_BYTES / WORD_BYTES;   // 8
  parameter int unsigned NUM_LINES    = CACHE_SIZE / LINE_BYTES;   // 256
  parameter int unsigned NUM_SETS     = NUM_LINES / WAYS;          // 64

  parameter int unsigned OFFSET_BITS  = $clog2(LINE_BYTES);        // 6
  parameter int unsigned WOFF_BITS    = $clog2(WORDS_PER_LN);      // 3 (word in line)
  parameter int unsigned BYTE_BITS    = $clog2(WORD_BYTES);        // 3 (byte in word)
  parameter int unsigned SET_BITS     = $clog2(NUM_SETS);          // 6
  parameter int unsigned WAY_BITS     = $clog2(WAYS);              // 2
  parameter int unsigned TAG_BITS     = ADDR_WIDTH - SET_BITS - OFFSET_BITS; // 20

  // AXI burst length for a full line fill/evict (beats) and the AxLEN encoding.
  parameter int unsigned BURST_BEATS  = WORDS_PER_LN;             // 8
  parameter int unsigned AXLEN        = BURST_BEATS - 1;          // 7

  // --------------------------------------------------------------------------
  // Convenience scalar types
  // --------------------------------------------------------------------------
  typedef logic [ADDR_WIDTH-1:0] addr_t;
  typedef logic [DATA_WIDTH-1:0] data_t;
  typedef logic [WORD_BYTES-1:0] strb_t;
  typedef logic [LINE_BITS-1:0]  line_t;
  typedef logic [LINE_BYTES-1:0] linestrb_t;
  typedef logic [TAG_BITS-1:0]   tag_t;
  typedef logic [SET_BITS-1:0]   set_t;
  typedef logic [WAY_BITS-1:0]   way_t;
  typedef logic [WOFF_BITS-1:0]  woff_t;
  typedef logic [CPU_ID_W-1:0]   cpu_id_t;
  typedef logic [AXI_ID_W-1:0]   axi_id_t;

  // Address field extraction helpers. The byte address is split:
  //   [ TAG | SET | WORD-OFFSET | BYTE-OFFSET ]
  function automatic tag_t  addr_tag (input addr_t a);
    return a[ADDR_WIDTH-1 -: TAG_BITS];
  endfunction
  function automatic set_t  addr_set (input addr_t a);
    return a[OFFSET_BITS +: SET_BITS];
  endfunction
  function automatic woff_t addr_woff(input addr_t a);
    return a[BYTE_BITS +: WOFF_BITS];
  endfunction
  // The line-aligned base address (tag+set, offset zeroed).
  function automatic addr_t addr_line_base(input addr_t a);
    addr_t m;
    m = a;
    m[OFFSET_BITS-1:0] = '0;
    return m;
  endfunction

  // --------------------------------------------------------------------------
  // CPU-side request / response payloads (valid/ready, tagged, OoO responses)
  // --------------------------------------------------------------------------
  typedef struct packed {
    cpu_id_t id;     // tag echoed on the response; lets responses return OoO
    addr_t   addr;   // byte address
    logic    we;     // 1 = store, 0 = load
    data_t   wdata;  // store data (word granular)
    strb_t   wstrb;  // store byte strobes
  } cpu_req_t;

  typedef struct packed {
    cpu_id_t id;     // matches the request id
    data_t   rdata;  // load data (don't-care for stores)
    logic    err;    // reserved (slave error); always 0 in this design
  } cpu_rsp_t;

  // --------------------------------------------------------------------------
  // pLRU (tree) helpers for a 4-way set. 3 state bits per set:
  //   b0 : 0 => victim in {w0,w1} group, 1 => victim in {w2,w3} group
  //   b1 : 0 => victim w0, 1 => victim w1   (left group)
  //   b2 : 0 => victim w2, 1 => victim w3   (right group)
  // Bits point *towards* the LRU side; on access they are flipped to point away
  // from the just-touched way.
  // --------------------------------------------------------------------------
  // MSHR per-entry lifecycle state (in the package so SVA/formal binds can
  // reference the type).
  typedef enum logic [2:0] {
    E_EMPTY, E_EVICT, E_FILL, E_FILLW, E_REFILL_REQ, E_WBWAIT
  } mshr_estate_e;

  parameter int unsigned PLRU_BITS = WAYS - 1; // 3 for 4-way

  // Victim way implied by the current pLRU state (ignores MSHR locks; the top
  // applies lock masking on top of this).
  function automatic way_t plru_victim(input logic [PLRU_BITS-1:0] s);
    if (s[0] == 1'b0) return (s[1] == 1'b0) ? way_t'(0) : way_t'(1);
    else              return (s[2] == 1'b0) ? way_t'(2) : way_t'(3);
  endfunction

  // Next pLRU state after touching `w`.
  function automatic logic [PLRU_BITS-1:0] plru_update(
      input logic [PLRU_BITS-1:0] s, input way_t w);
    logic [PLRU_BITS-1:0] n;
    n = s;
    unique case (w)
      2'd0: begin n[0] = 1'b1; n[1] = 1'b1; end
      2'd1: begin n[0] = 1'b1; n[1] = 1'b0; end
      2'd2: begin n[0] = 1'b0; n[2] = 1'b1; end
      2'd3: begin n[0] = 1'b0; n[2] = 1'b0; end
      default: ;
    endcase
    return n;
  endfunction

endpackage : nb_dcache_pkg

`endif
