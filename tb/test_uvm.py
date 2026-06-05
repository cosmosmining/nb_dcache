"""pyuvm tests for the non-blocking L1 data cache.

A base test owns bring-up (clock, reset, AXI slave, monitors) and teardown
(killing background tasks so the next test in the same simulation starts clean),
the scoreboard check, and a coverage dump. Each concrete test selects a sequence
and tunes the configuration.

Run all:        make sim            (or: make -C tb MODULE=test_uvm)
Run one:        make -C tb MODULE=test_uvm COCOTB_TESTCASE=RandomTest
"""

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles
import pyuvm
from pyuvm import uvm_test, ConfigDB

from cache_cfg import CacheConfig
from cache_bfm import CpuBfm
from cache_components import CacheEnv
from axi_slave import AxiSlaveMemory
import cache_sequences as seqs


class CacheBaseTest(uvm_test):
    """Override `make_cfg()` and `seq_cls` in subclasses."""
    seq_cls = seqs.RandomSeq

    def make_cfg(self):
        return CacheConfig()

    def build_phase(self):
        self.cfg = self.make_cfg()
        self.bfm = CpuBfm(cocotb.top)
        ConfigDB().set(None, "*", "cfg", self.cfg)
        ConfigDB().set(None, "*", "bfm", self.bfm)
        self.env = CacheEnv("env", self)
        self._tasks = []

    async def run_phase(self):
        self.raise_objection()
        dut = cocotb.top

        await self.bfm.start_clock()
        await self.bfm.reset()
        slave = AxiSlaveMemory(dut, max_latency=self.cfg.axi_max_latency,
                               seed=self.cfg.seed)
        self._tasks += slave.start()
        self._tasks += self.bfm.start_monitors()

        seq = self.seq_cls("seq")
        await seq.start(self.env.agent.seqr)

        # drain: wait until every id has been recycled (all requests retired)
        for _ in range(500000):
            if self.cfg.free_ids.qsize() == self.cfg.n_ids:
                break
            await RisingEdge(dut.clk)
        await ClockCycles(dut.clk, 40)   # let occupancy settle to 0

        # tear down background tasks so the next test starts clean
        for t in self._tasks:
            t.kill()
        self._tasks = []
        self.drop_objection()

    def report_phase(self):
        self.env.sb.check()
        self.logger.info("retired=%d checked=%d", self.env.sb.retired,
                         self.env.sb.checks)
        self.logger.info("coverage: %s", self.env.cov.summary())


# ---------------------------------------------------------------------------
# Concrete tests
# ---------------------------------------------------------------------------
@pyuvm.test()
class HitUnderMissTest(CacheBaseTest):
    seq_cls = seqs.HitUnderMissSeq

    def make_cfg(self):
        c = CacheConfig(); c.axi_max_latency = 30; c.seed = 11; return c


@pyuvm.test()
class MergeTest(CacheBaseTest):
    seq_cls = seqs.MergeSeq

    def make_cfg(self):
        c = CacheConfig(); c.axi_max_latency = 50; c.seed = 21; return c


@pyuvm.test()
class FullPressureTest(CacheBaseTest):
    seq_cls = seqs.FullPressureSeq

    def make_cfg(self):
        c = CacheConfig(); c.axi_max_latency = 40; c.seed = 31; return c


@pyuvm.test()
class EvictDuringRefillTest(CacheBaseTest):
    seq_cls = seqs.EvictDuringRefillSeq

    def make_cfg(self):
        c = CacheConfig(); c.axi_max_latency = 35; c.seed = 32; return c


@pyuvm.test()
class RandomTest(CacheBaseTest):
    seq_cls = seqs.RandomSeq

    def make_cfg(self):
        c = CacheConfig()
        c.sets = [0, 1, 2]; c.tags = list(range(8))
        c.n_items = 1500; c.axi_max_latency = 16; c.seed = 101
        return c


@pyuvm.test()
class RandomSingleSetTest(CacheBaseTest):
    seq_cls = seqs.RandomSeq

    def make_cfg(self):
        c = CacheConfig()
        c.sets = [5]; c.tags = list(range(8))
        c.n_items = 1200; c.axi_max_latency = 24; c.seed = 202
        return c
