"""Functional-coverage closure test.

Runs a varied workload designed to hit every required coverage bin (MSHR
occupancy 0..4, merge, replay, dirty/clean evictions, response reordering, and
each hazard case) and asserts closure at the end.
"""

import random
import cocotb
from cocotb.triggers import RisingEdge

from tb_common import setup_nb, addr_for, WORD_BYTES
from coverage import Coverage


@cocotb.test()
async def test_coverage_closure(dut):
    cov = Coverage(dut)
    # high AXI latency -> fills stay in flight -> occupancy grows, replays,
    # reordering, merge windows all become reachable.
    c = await setup_nb(dut, seed=77, max_latency=40, coverage=cov)
    a = c.agent
    rng = random.Random(77)

    # Phase 1: hammer ONE set with > ways tags and high latency -> full MSHRs,
    # set-locked replays, dirty + clean evictions, merges.
    for _ in range(400):
        s = rng.choice([0, 1])
        t = rng.choice(range(10))
        w = rng.randrange(8)
        a_addr = addr_for(s, t, w)
        if rng.random() < 0.5:
            await a.issue(a_addr, we=1, wdata=rng.getrandbits(64),
                          wstrb=rng.choice([0xFF, 0x0F, 0xAA]))
        else:
            await a.issue(a_addr, we=0)

    # Phase 2: many same-line accesses during a slow fill -> merge-window + wb
    # conflict hazards, deep occupancy.
    for rep in range(40):
        base = addr_for(set_idx=2, tag=rep % 6)
        for w in range(8):
            await a.issue(base + w * WORD_BYTES, we=(w & 1), wdata=0xC0DE + w)

    await a.drain()

    # let the pipeline settle back to occupancy 0
    for _ in range(50):
        await RisingEdge(dut.clk)

    c.sb.report()
    cocotb.log.info("coverage summary: %s", cov.summary())
    cov.assert_closed()
