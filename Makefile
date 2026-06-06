# ============================================================================
# nb_dcache top-level Makefile
# ----------------------------------------------------------------------------
#   make sim     - run the full cocotb/pyuvm regression + coverage
#   make lint    - Verilator lint (RTL must be clean)
#   make formal  - SymbiYosys bounded proofs on the MSHR file
#   make clean    - remove build artifacts
#
# Uses the project virtualenv in ./.venv for cocotb/pyuvm.
# ============================================================================
ROOT    := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
VENV    := $(ROOT)/.venv
RTL     := $(ROOT)/rtl
BIN     := $(VENV)/bin

# All RTL for lint (order: package consumers; pkg pulled in via `include`).
LINT_SRCS := \
	$(RTL)/nb_dcache_tag_array.sv \
	$(RTL)/nb_dcache_data_array.sv \
	$(RTL)/nb_dcache_mshr.sv \
	$(RTL)/nb_dcache_replayq.sv \
	$(RTL)/nb_dcache_axi.sv \
	$(RTL)/nb_dcache_maint.sv \
	$(RTL)/nb_dcache_top.sv

# Regression modules. The pyuvm suite (test_uvm) is the primary verification
# environment; the staged directed tests document the incremental bring-up; the
# coverage test closes functional coverage. Each runs in its OWN simulation so
# per-test background tasks never cross-contaminate.
MODULES := test_stage1 test_stage2 test_stage3 test_stage4 \
           test_random test_errors test_maint test_throughput \
           test_coverage test_uvm

.PHONY: sim uvm lint formal synth clean

SYNTH_SRCS := $(RTL)/nb_dcache_pkg.sv $(RTL)/nb_dcache_tag_array.sv \
	$(RTL)/nb_dcache_data_array.sv $(RTL)/nb_dcache_mshr.sv \
	$(RTL)/nb_dcache_replayq.sv $(RTL)/nb_dcache_axi.sv \
	$(RTL)/nb_dcache_maint.sv $(RTL)/nb_dcache_top.sv

lint:
	verilator --lint-only -Wall -sv -I$(RTL) $(RTL)/nb_dcache.vlt \
		$(RTL)/nb_dcache_top.sv --top-module nb_dcache_top
	@echo "LINT CLEAN"

sim:
	@fail=0; for m in $(MODULES); do \
	  echo "==================== $$m ===================="; \
	  rm -rf tb/sim_build; \
	  PATH="$(BIN):$$PATH" $(MAKE) -C tb MODULE=$$m || fail=1; \
	done; \
	if [ $$fail -ne 0 ]; then echo "*** SIM FAILED ***"; exit 1; \
	else echo "*** ALL SIM PASSED ***"; fi

# Just the professional pyuvm regression.
uvm:
	rm -rf tb/sim_build; PATH="$(BIN):$$PATH" $(MAKE) -C tb MODULE=test_uvm

formal:
	sv2v -I$(RTL) $(RTL)/nb_dcache_pkg.sv $(RTL)/nb_dcache_mshr.sv \
		formal/mshr_formal.sv > formal/mshr_formal_sv2v.v
	cd formal && sby -f mshr_proofs.sby

synth:
	mkdir -p synth
	sv2v -I$(RTL) $(SYNTH_SRCS) > synth/nb_dcache_synth.v
	cd synth && yosys synth.ys | tee synth.log | sed -n '/Number of cells/,/End of script/p'

clean:
	rm -rf tb/sim_build tb/__pycache__ tb/results.xml formal/mshr_proofs \
		$(RTL)/*.vcd tb/*.vcd
