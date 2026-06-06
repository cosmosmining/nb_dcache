# nb_dcache — Sign-off Results

Open-source flow: Verilator 5.048 · cocotb 2.0.1 + pyuvm 4.0.1 · Yosys 0.66 +
SymbiYosys + z3 · sv2v. Reproduce with `make lint && make sim && make formal &&
make synth`. Raw per-step summaries are alongside this file in `results/`.

## Lint — CLEAN
`make lint` → Verilator `-Wall` clean (intentional unused-bit cases waived in
`rtl/nb_dcache.vlt`). See `results/lint.log`.

## Simulation — 32/32 PASS (9 modules)
`make sim`, each module in its own simulation, SVA bound (`--assert`).

| Module | Tests | Result |
|--------|------:|--------|
| test_stage1 (blocking baseline) | 5 | PASS |
| test_stage2 (hit-under-miss) | 2 | PASS |
| test_stage3 (miss-under-miss + merge) | 5 | PASS |
| test_stage4 (replay / hazards / reset) | 4 | PASS |
| test_random (constrained-random stress) | 4 | PASS |
| test_errors (AXI bus-error propagation) | 2 | PASS |
| test_maint (invalidate / flush) | 3 | PASS |
| test_coverage (functional-coverage closure) | 1 | PASS |
| test_uvm (pyuvm suite) | 6 | PASS |
| **Total** | **32** | **PASS** |

See `results/sim_summary.txt`.

## Functional coverage — CLOSED
All required bins hit (`results/coverage_summary.txt`):

* MSHR occupancy: {0, 1, 2, 3, 4}
* merge events, replays, dirty evictions, clean evictions: all > 0
* observed response reordering: > 0
* hazard bins: mshr_full, no_free_way, wb_conflict, merge_window — all > 0

## Formal — PASS (bmc + cover)
`make formal` — SymbiYosys bounded model checking (depth 20) + cover on the MSHR
file (`formal/mshr_formal.sv`), via sv2v + yosys + z3.

* `mshr_proofs_bmc`  → **PASS** — address uniqueness, free-slot / no-alloc-when-
  full, waiter conservation (exactly-once retirement).
* `mshr_proofs_cover` → **PASS** — response, full, merge, write-back-wait states
  all reachable (proofs are non-vacuous).

See `results/formal.log`.

## Synthesis — SRAM-mappable (synchronous-read arrays)
`make synth` — sv2v → yosys generic synthesis (`synth/synth.ys`). The arrays are
now **synchronous-read**, so the data array infers as a real **memory** ($mem,
SRAM-mappable) instead of flip-flops. Cells per block (incl. submodules),
`results/synth_summary.txt`:

| Block | Cells | Note |
|-------|------:|------|
| `nb_dcache_data_array` | ~2.3k + **1 SRAM** | 16 KiB (131072 b) as one `$mem` |
| `nb_dcache_tag_array`  | ~5.4k | tag/valid/dirty/pLRU as flops (~5 Kbit) |
| `nb_dcache_mshr`       | ~12.1k | MSHR file + waiter/merge storage |
| `nb_dcache_axi`        | ~0.2k | (+ small per-ID buffers) |
| `nb_dcache_replayq`    | ~0.05k |
| `nb_dcache_maint`      | ~0.05k |
| **top total**          | **~20.4k cells + data SRAM** |

Compared with the earlier async-read register-file version (~670k cells, ~150k
flip-flops), the 16 KiB data array is now a single SRAM-mappable memory. An
SRAM-targeted backend (`memory_libmap`) maps the `$mem` to a compiled macro.
Lookup is a two-phase (address → decide) non-pipelined sequence; pipelining to
1 access/cycle with cross-phase forwarding is the next performance step.
