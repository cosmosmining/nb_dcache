"""Shared bring-up helpers for nb_dcache cocotb tests."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from axi_slave import AxiSlaveMemory
from cpu_iface import CpuDriver, CpuMonitor
from ref_model import RefModel


# Geometry mirrors nb_dcache_pkg defaults (16 KiB, 64 B line, 4 way).
LINE_BYTES = 64
WORD_BYTES = 8
WORDS_PER_LINE = LINE_BYTES // WORD_BYTES
NUM_SETS = 64
WAYS = 4
SET_SHIFT = 6                       # log2(LINE_BYTES)


def set_of(addr):
    return (addr >> SET_SHIFT) & (NUM_SETS - 1)


def line_base(addr):
    return addr & ~(LINE_BYTES - 1)


def addr_for(set_idx, tag, word=0):
    """Build an address landing in `set_idx` with a given tag and word offset."""
    return (tag << (SET_SHIFT + 6)) | (set_idx << SET_SHIFT) | (word * WORD_BYTES)


async def setup(dut, seed=1, max_latency=12):
    """Start clock, AXI slave, monitors and apply reset. Returns a context."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    drv = CpuDriver(dut)
    mon = CpuMonitor(dut)
    slave = AxiSlaveMemory(dut, max_latency=max_latency, seed=seed)
    ref = RefModel(WORD_BYTES)

    await drv.reset_inputs()
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    slave.start()
    mon.start()

    class Ctx:
        pass
    c = Ctx()
    c.drv, c.mon, c.slave, c.ref = drv, mon, slave, ref
    return c


async def setup_nb(dut, seed=1, max_latency=12, coverage=None):
    """Concurrent (non-blocking) bring-up: returns an env with a CpuAgent."""
    from env import CpuAgent, Scoreboard
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.cpu_req_valid.value = 0
    dut.cpu_req_id.value = 0
    dut.cpu_req_addr.value = 0
    dut.cpu_req_we.value = 0
    dut.cpu_req_wdata.value = 0
    dut.cpu_req_wstrb.value = 0
    dut.cpu_rsp_ready.value = 1
    dut.maint_req_valid.value = 0
    dut.maint_req_flush.value = 0

    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    slave = AxiSlaveMemory(dut, max_latency=max_latency, seed=seed)
    slave.start()

    sb = Scoreboard()
    agent = CpuAgent(dut, sb, coverage=coverage)
    agent.start()
    if coverage is not None:
        coverage.start()

    class Ctx:
        pass
    c = Ctx()
    c.agent, c.sb, c.slave = agent, sb, slave
    return c
