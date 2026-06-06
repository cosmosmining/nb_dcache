# nb_dcache — Non-Blocking L1 Data Cache

A synthesizable, parameterized, **non-blocking** L1 data cache in SystemVerilog
with a full open-source verification environment (Verilator + cocotb + pyuvm),
bound SVA assertions, and SymbiYosys formal proofs.

* 4-way set-associative, tree-pLRU replacement
* **Synchronous-read, SRAM-mappable** tag/data arrays (way-parallel) with a
  **pipelined lookup sustaining 1 hit/cycle** (address-prefetch → decide, with
  a 1-deep write bypass)
* Write-back, write-allocate
* 4 MSHRs: **hit-under-miss** and **miss-under-miss**, with secondary-miss
  merging (per-word sub-entries), store-to-load forwarding in the fill window,
  and write merging into in-flight MSHRs
* CPU side: tagged valid/ready load/store interface with **out-of-order**
  responses
* Memory side: **AXI4** master — burst line fills and dirty evictions, multiple
  outstanding transactions via ARID/AWID, correct reassembly of out-of-order R
  responses across IDs, full sideband (`AxPROT/AxCACHE/AxLOCK/AxQOS`)
* Explicit stall/**replay** policy for structural hazards
* **Bus-error handling**: `RRESP/BRESP` SLVERR/DECERR propagated to
  `cpu_rsp_err`; an errored line is **not installed** (CPU can retry)
* **Cache-maintenance** ops: invalidate-all and flush-all (write-back + invalidate)
* **Performance counters**: hits, misses, replays, write-backs, bus errors

Default geometry: `CACHE_SIZE=16 KiB`, `LINE_BYTES=64`, `WAYS=4`,
`DATA_WIDTH=64` ⇒ 256 lines, 64 sets, 8 words/line, 8-beat AXI bursts. All
derived from three knobs in `rtl/nb_dcache_pkg.sv`.

---

## Block diagram

```
                         CPU load/store port
        cpu_req (id,addr,we,wdata,wstrb) valid/ready │ cpu_rsp (id,rdata) valid/ready
                                  │                  ▲   (tagged, out-of-order)
                                  ▼                  │
   ┌───────────────────────────────────────────────────────────────────────┐
   │ nb_dcache_top                                                           │
   │                                                                        │
   │   replay queue ─► [input arb] ─► single-cycle LOOKUP                    │
   │   (HoL, in-order)      ▲              │  tag compare + pLRU victim      │
   │        ▲               │              │  (victim masked to UNLOCKED     │
   │        │ push hazard   │              │   ways via MSHR lock mask)      │
   │        └───────────────┴───────┐      │                                 │
   │                                │      ├─ HIT  ─► data array ─► response  │
   │   ┌────────────┐   ┌───────────┴───┐  │                                 │
   │   │ tag array  │   │  MSHR file /  │  ├─ MISS ─► alloc / merge ─► MSHR   │
   │   │ tag/valid/ │◄─►│  miss engine  │◄─┘                                 │
   │   │ dirty/pLRU │   │  (4 entries,  │   refill ctrl: write line + alloc  │
   │   └────────────┘   │  waiter FIFOs)│   tag, then DRAIN waiters ─► resp  │
   │   ┌────────────┐   └───────┬───────┘   (block_lookup during those few   │
   │   │ data array │◄──────────┘            cycles -> hit path stalls)      │
   │   │ (line RAM, │                                                        │
   │   │ byte-WE)   │           │ fill_req/rsp        │ evict_req/done        │
   │   └────────────┘           ▼                     ▼                       │
   │                    ┌──────────────────────────────────────┐            │
   │                    │ nb_dcache_axi (AXI4 master engine)    │            │
   │                    │  AR/R burst fills (per-ID reassembly, │            │
   │                    │  OoO across IDs); AW/W/B dirty evicts │            │
   │                    └──────────────────────────────────────┘            │
   └───────────────────────────────────│──────────────────────────────────┘
                                        ▼  AXI4  (to next-level memory)
```

Module split (`rtl/`):

| File | Role |
|------|------|
| `nb_dcache_pkg.sv`        | parameters, types, address-field & pLRU helpers |
| `nb_dcache_tag_array.sv`  | tag / valid / dirty + per-set tree-pLRU (synchronous read) |
| `nb_dcache_data_array.sv` | **SRAM-mappable** line data RAM: synchronous, way-parallel read, byte-write |
| `nb_dcache_mshr.sv`       | **miss engine**: lookup, alloc, merge, AXI phases, refill+drain |
| `nb_dcache_replayq.sv`    | replay FIFO (head-of-line) for structural hazards |
| `nb_dcache_axi.sv`        | AXI4 master: burst fills/evicts, multi-outstanding, OoO R/B, error capture |
| `nb_dcache_maint.sv`      | maintenance engine: invalidate-all / flush-all walk |
| `nb_dcache_top.sv`        | hit path + arbitration + integration + perf counters |

---

## MSHR lifecycle (per entry)

```
                 alloc (primary miss; victim invalidated, victim line captured)
                    │
         need_evict?├──────────────► E_EVICT ──(evict_req accepted)──┐
                    │  no                                            │
                    └────────────────────────────────────────► E_FILL
                                                                     │
                                            (fill_req accepted)      ▼
                                                                  E_FILLW
                                              (fill_rsp: capture memory line)
                                                                     │
                                                                     ▼
                                                              E_REFILL_REQ ──┐
                              refill controller grants (one entry at a time):│
                              RC_WRITE: write {memory ⊕ merged stores} to     │
                                        data array; allocate tag; pLRU touch  │
                              RC_DRAIN: emit one response per waiter           │
                                                                     │ drained
                                                ┌────────────────────┘
                          write-back B done? ──►│
                            yes                 │   no (B still outstanding)
                              ▼                 ▼
                           E_EMPTY  ◄──(B)── E_WBWAIT   (entry held: way locked,
                                                         evicted-line tracked to
                                                         block WAR re-access)
```

* **Waiters** are parked per entry in a small FIFO. A **store** waiter merges its
  bytes into the entry's per-word `wr_data/wr_strb` and parks a store-ack. A
  **load** waiter snapshots the currently-merged store bytes for its word *at
  enqueue time*, so program order is preserved per byte (a load before a store
  reads memory; a load after a store forwards the store).
* `block_lookup` is asserted only during `RC_WRITE`/`RC_DRAIN` (a few cycles),
  so the hit path runs during the long AXI latency ⇒ hit-under-miss.

---

## Hazard table

| # | Corner case | Where | Resolution mechanism |
|---|-------------|-------|----------------------|
| 1 | **Store-to-load forwarding in the fill window** | `mshr` merge | A load merged into an in-flight MSHR snapshots the merged store bytes for its word at enqueue (`w_fdata/w_fstrb`); on drain the result is `memory ⊕ snapshot`. |
| 2 | **Write merging into an in-flight MSHR** | `mshr` merge | A store to a line with an open MSHR merges bytes into per-word `wr_data/wr_strb` (last-writer-per-byte) and parks a store-ack; applied to the line at `RC_WRITE`. |
| 3 | **Back-to-back same-line misses** | `top` lookup | Tag miss + MSHR line hit + mergeable ⇒ merge as a secondary waiter; **never** a second MSHR for the same line. |
| 4 | **Eviction targeting a set with a pending miss** | `top` victim sel | Victim is chosen only from ways **not** locked by an MSHR (`look_locked_ways`); if all ways locked ⇒ replay. |
| 5 | **Store to a line being evicted (fill window)** | `top` alloc | The victim's tag is **invalidated at allocation** so the doomed line cannot be hit/modified after its data was captured for write-back ⇒ no lost store. |
| 6 | **WAR: re-access an evicted line before its write-back lands** | `mshr` `look_wb_conflict` | A primary miss whose line equals an in-flight eviction's address is **replayed** until the eviction's `B` completes (entry held in `E_WBWAIT`). |
| 7 | **MSHR file full** | `top`/`replayq` | Primary miss with no free entry ⇒ pushed to the replay queue. |
| 8 | **Waiter FIFO full / past merge window** | `top`/`replayq` | MSHR line hit but not mergeable ⇒ replay until it resolves into a cache hit. |
| 9 | **Refill vs hit on shared ports** | `top` arb | `block_lookup` gives the refill controller priority for the data-write/tag-alloc/response ports for the few cycles it needs; the hit path stalls. |
| 10 | **OoO AXI R responses across IDs** | `axi` | Per-ID line buffers + beat counters; each ARID's beats reassemble independently; whole bursts may return out of order. |
| 11 | **Single-cycle hit accept+respond** | scoreboard/SVA | Retirement checker allows a response in the same cycle as its accept. |
| 12 | **Reset mid-burst** | TB/SVA | Cache recovers; AXI orphan check uses global AR-vs-R accounting that survives the DUT reset (the slave still completes accepted transactions). |

---

## Verification

`tb/` — cocotb + **pyuvm** environment (open-source):

* **Reference model** (`ref_model.py`) — architectural flat-memory semantics,
  **not** a microarchitectural mirror. Applied in program (acceptance) order;
  load expectations snapshotted at issue.
* **AXI slave model** (`axi_slave.py`) — randomized latency, **out-of-order**
  completion across IDs (transaction-granular, AXI4-legal).
* **pyuvm env** (`cache_*.py`, `test_uvm.py`) — BFM, passive request/response
  monitors, sequencer + driver, reference-model scoreboard, coverage subscriber,
  config object, sequence library, and a set of `uvm_test`s.
* **Staged directed tests** (`test_stage1..4.py`) — document the incremental
  bring-up (blocking → hit-under-miss → miss-under-miss/merge → replay/hazards).
* **Constrained-random** stress biased to a few sets (`test_random.py`).
* **Functional coverage** (`coverage.py`, `test_coverage.py`) — MSHR occupancy
  0–4, merge/replay events, dirty vs clean evictions, observed response
  reordering, one bin per hazard case; `test_coverage` asserts closure.

Assertions (`sva/nb_dcache_sva.sv`, bound, enabled with Verilator `--assert`):

* no two MSHRs hold the same line address
* AXI legality — AR/AW/W stable until handshake; no orphan R/B; bounded,
  closing-out ID bookkeeping
* every accepted request retires exactly once
* pLRU never evicts a way reserved by an active MSHR

Formal (`formal/`, SymbiYosys + yosys + z3, via `sv2v`):

* standalone bounded proofs on the MSHR file — address uniqueness, free-slot /
  no-allocate-when-full, and exactly-once retirement (waiter conservation);
  plus cover tasks proving the invariants are non-vacuous.

### Running

```sh
make lint      # Verilator lint (RTL must be clean)
make sim       # full regression: staged + random + errors + maint + coverage + pyuvm
make uvm       # just the pyuvm regression
make formal    # SymbiYosys bounded proofs + cover on the MSHR file
make synth     # sv2v + yosys generic synthesis, gate/area report
```

Regression modules: `test_stage1..4` (incremental bring-up), `test_random`
(constrained-random stress), `test_errors` (AXI bus-error propagation),
`test_maint` (invalidate/flush), `test_throughput` (1 hit/cycle pipelining),
`test_coverage` (functional-coverage closure), `test_uvm` (the pyuvm suite).

Latest sign-off results (lint / 32-test regression / coverage / formal /
synthesis) are captured in [`results/SIGNOFF.md`](results/SIGNOFF.md).

The Python tools live in `./.venv` (cocotb 2.x, pyuvm 4.x); the Makefiles put it
on `PATH` automatically.

---

## Synthesis & roadmap to production silicon

`make synth` converts the RTL with `sv2v` and runs a generic yosys synthesis
(`synth/synth.ys`). The tag/data arrays are **synchronous-read**, so the 16 KiB
data array infers as a real **SRAM** (`$mem`) instead of flip-flops. Cells per
block (incl. submodules, default 16 KiB geometry):

| Block | Cells | Note |
|-------|------:|------|
| `nb_dcache_data_array` | ~2.3k + **1 SRAM** | 16 KiB (131072 b) as one `$mem` |
| `nb_dcache_tag_array`  | ~5.4k | tag/valid/dirty/pLRU as flops (~5 Kbit) |
| `nb_dcache_mshr`       | ~12.1k | MSHR file + waiter/merge storage |
| `nb_dcache_axi`        | ~0.2k | (+ small per-ID buffers) |
| `nb_dcache_replayq`    | ~0.05k | replay FIFO |
| `nb_dcache_maint`      | ~0.05k | maintenance walker |
| **top total** | **~20.4k cells + data SRAM** | vs ~670k when arrays were async-flops |

Lookup is a **pipelined, synchronous-read** S1→S2 sequence sustaining
**1 access/cycle** (proven by `test_throughput`: 64 back-to-back hits in 64
cycles): S1 prefetches the next request's set while S2 decides the current one.
Correctness under overlap is maintained by

* a **1-deep write bypass** patching the one write a same-posedge read can miss
  (previous commit's store data / dirty / victim invalidate),
* a **stale re-read** of S2's set after any refill rewrites the arrays,
* an **advance veto on hazard pushes** plus serialized replay execution, so
  program order is never reordered past a stalled request.

An SRAM-targeted backend (`memory_libmap`) maps the `$mem` to a compiled macro.

Remaining roadmap to production silicon:

1. **Critical-word-first / early restart** on fills (lower miss latency).
2. **ECC/parity on tag & data** + poison/scrub for RAS.
3. **Power intent** (UPF, clock-gating on idle ways, SRAM light-sleep) and
   **DFT** (MBIST for the SRAMs, scan).
4. **AXI compliance VIP** in regression; multi-seed CI; code/toggle coverage;
   gate-level sim with SDF.
5. **Maintenance by line/set** and CSR/APB programming interface for the
   maintenance ops and performance counters.

**Already implemented** (productization done in this repo):
**synchronous-read SRAM-mappable tag/data arrays** with a **1-access/cycle
pipelined lookup** (write bypass + stale re-read + ordering guards),
AXI4 **error propagation** (`RRESP/BRESP`→`cpu_rsp_err`, errored line not
installed), full AXI4 **sideband**, **performance counters**, and
**cache-maintenance** (invalidate-all / flush-all).

## 12 interview questions this design answers

1. **What is an MSHR and what minimum state must it hold?** A Miss-Status
   Holding Register tracks an outstanding miss: line address, the victim
   way/eviction info, the captured fill data, and the list of waiting requests
   (here, per-word merged store bytes + a waiter FIFO).
2. **Hit-under-miss vs miss-under-miss — how does the structure differ?**
   Hit-under-miss needs only that misses don't block the hit path (one MSHR
   suffices). Miss-under-miss needs *multiple* MSHRs and an AXI engine with
   multiple outstanding transactions tagged by ID.
3. **How are secondary misses to the same line merged, and why not allocate a
   second MSHR?** A second MSHR for the same line would create two fills and a
   coherence ambiguity; instead the request is appended as a waiter to the
   existing MSHR (per-word sub-entries). Uniqueness is an assertion *and* a
   formal proof here.
4. **How do you forward a store to a later load while the line is still being
   filled?** The load snapshots the MSHR's currently-merged store bytes for its
   word at enqueue; on refill the result is `memory ⊕ snapshot`, preserving
   per-byte program order.
5. **Write-back + write-allocate: what happens on a store miss?** Allocate an
   MSHR, fetch the line, merge the store into it as it lands, mark the line
   dirty. A dirty victim is written back first.
6. **What's the WAR hazard between an eviction and a later fill of the same
   address?** If a fill (read) of an evicted line passes its write-back (write)
   at memory, you read stale data. Resolved by tracking in-flight write-backs
   (`look_wb_conflict`/`E_WBWAIT`) and replaying conflicting fills until `B`.
7. **Why invalidate the victim's tag at allocation rather than at refill?**
   Otherwise the doomed line stays hittable during the fill window and a store
   to it would be lost (its data was already captured for write-back).
8. **How does pLRU avoid evicting a line that has an in-flight miss?** Victim
   selection masks out ways reserved by MSHRs in that set; if all are locked the
   request replays. Proven by the `pLRU never evicts a locked way` assertion.
9. **How does an AXI master reassemble out-of-order read data across IDs?**
   AXI4 forbids beat-level interleaving, so each ARID's burst is contiguous;
   per-ID line buffers + beat counters let whole bursts complete out of order
   and still reassemble (`nb_dcache_axi`).
10. **What structural hazards force a stall/replay, and how do you avoid
    deadlock?** MSHRs full, all ways locked, waiter FIFO full, or a write-back
    conflict. The replay queue retries in order with head-of-line blocking;
    progress is guaranteed because in-flight fills always complete (AXI always
    responds).
11. **How do you guarantee every request retires exactly once with OoO,
    tagged responses?** Unique ids while outstanding; the scoreboard checks by
    id; an SVA tracks a per-id pending bit (set on accept, cleared on response —
    no double-accept, no orphan response).
12. **How would you formally prove the miss-handling logic?** Drive the MSHR
    with unconstrained stimulus, *assume* the top's contract (alloc only into a
    free slot / a not-yet-tracked line, merge only into the matching mergeable
    entry), and *assert* uniqueness, free-slot validity, and waiter conservation
    (enqueues − dequeues = Σ occupancy) ⇒ exactly-once. See `formal/`.
