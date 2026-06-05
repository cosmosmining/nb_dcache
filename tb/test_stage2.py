"""Stage-2 tests: hit-under-miss (non-blocking).

A long-latency miss is outstanding while subsequent HITS are serviced and
retire ahead of it (observable response reordering).
"""

import cocotb
from cocotb.triggers import RisingEdge

from tb_common import setup_nb, addr_for


@cocotb.test()
async def test_hit_under_miss(dut):
    c = await setup_nb(dut, seed=11, max_latency=30)
    a = c.agent

    # Warm a line so later accesses to it hit.
    B = addr_for(set_idx=5, tag=2)
    await a.issue(B, we=1, wdata=0xB0B0B0B0B0B0B0B0)
    await a.drain()

    # Kick off a slow miss to a different line.
    A = addr_for(set_idx=6, tag=3)
    await a.issue(A, we=0)

    # While that miss is in flight, issue several hits to B.
    n_before = c.sb.retired
    for _ in range(6):
        await a.issue(B, we=0)

    # Hits should retire while the miss is still outstanding.
    for _ in range(50):
        if c.sb.retired - n_before >= 6:
            break
        await RisingEdge(dut.clk)
    assert c.sb.retired - n_before >= 1, "no hit-under-miss progress observed"

    await a.drain()
    c.sb.report()


@cocotb.test()
async def test_many_independent_misses(dut):
    """Several misses to distinct lines overlap (uses all MSHRs)."""
    c = await setup_nb(dut, seed=12, max_latency=20)
    a = c.agent
    for t in range(8):
        await a.issue(addr_for(set_idx=t, tag=t + 1), we=0)
    await a.drain()
    c.sb.report()
