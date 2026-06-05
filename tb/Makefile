# cocotb runner for nb_dcache (Verilator backend).
#
# Usage (from this dir, with the project venv on PATH):
#   make            # run the default test module
#   make MODULE=test_stage1
#   make MODULE=test_stage1,test_stage2,...   # multiple modules
#
# The top-level ../Makefile wraps this with the venv and the full module list.

SIM            ?= verilator
TOPLEVEL_LANG  ?= verilog

RTL            := $(abspath ../rtl)
VERILOG_SOURCES := \
	$(RTL)/nb_dcache_tag_array.sv \
	$(RTL)/nb_dcache_data_array.sv \
	$(RTL)/nb_dcache_mshr.sv \
	$(RTL)/nb_dcache_replayq.sv \
	$(RTL)/nb_dcache_axi.sv \
	$(RTL)/nb_dcache_maint.sv \
	$(RTL)/nb_dcache_top.sv \
	$(abspath ../sva)/nb_dcache_sva.sv

TOPLEVEL       := nb_dcache_top
MODULE         ?= test_stage1

# -I for the package include; waive intentional unused-bit warnings; --assert
# turns the bound SVA into runtime checks.
COMPILE_ARGS   += -I$(RTL) -Wall -Wno-fatal
EXTRA_ARGS     += --trace --trace-structs --assert

export COCOTB_RESOLVE_X ?= ZEROS

include $(shell cocotb-config --makefiles)/Makefile.sim
