"""Stage-4 tests: structural-hazard replay + the nasty eviction corners.

  * sustained full-MSHR pressure  -> replay queue exercised
  * evict-during-refill           -> conflict misses in a hammered set
  * reset mid-burst               -> recovery
"""

import random
import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from tb_common import setup_nb, addr_for, WORD_BYTES


@cocotb.test()
async def test_full_mshr_pressure(dut):
    """More distinct in-flight miss lines than MSHRs -> forces replays."""
    c = await setup_nb(dut, seed=31, max_latency=40)
    a = c.agent
    # 16 distinct lines issued back-to-back; only 4 MSHRs -> replay queue churns
    for t in range(16):
        await a.issue(addr_for(set_idx=t % 8, tag=10 + t), we=(t & 1))
    await a.drain()
    c.sb.report()


@cocotb.test()
async def test_evict_during_refill(dut):
    """Hammer a single set with more tags than ways while misses are in flight,
    forcing dirty evictions concurrent with refills + WAR re-access."""
    c = await setup_nb(dut, seed=32, max_latency=35)
    a = c.agent
    s = 20
    # dirty up several tags (causes write-back evictions)
    for t in range(10):
        await a.issue(addr_for(s, t), we=1, wdata=0xE0000000_00000000 + t)
    # now read them all back -> many were evicted, re-accessed (WAR), refilled
    for t in range(10):
        await a.issue(addr_for(s, t), we=0)
    await a.drain()
    c.sb.report()


@cocotb.test()
async def test_back_to_back_same_line_misses(dut):
    """Streams of same-line misses interleaved across two sets under pressure."""
    c = await setup_nb(dut, seed=33, max_latency=45)
    a = c.agent
    for rep in range(4):
        for w in range(8):
            await a.issue(addr_for(1, 5, w), we=(w & 1), wdata=0xABCD0000 + w)
            await a.issue(addr_for(2, 6, w), we=0)
    await a.drain()
    c.sb.report()


@cocotb.test()
async def test_reset_mid_burst(dut):
    """Assert reset while transactions are in flight, then verify the cache
    recovers and serves fresh addresses correctly."""
    c = await setup_nb(dut, seed=34, max_latency=60)
    a = c.agent
    # launch a burst of slow misses, do NOT drain
    for t in range(6):
        await a.issue(addr_for(t, 40 + t), we=(t & 1), wdata=0x1234 + t)
    await ClockCycles(dut.clk, 8)   # mid-burst

    # yank reset (CPU req idle during reset)
    dut.cpu_req_valid.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # post-reset the cache is empty; clear in-flight bookkeeping and reuse the
    # same agent + slave (slave memory persists, fresh tags still read 0).
    a.reset_state()

    # fresh addresses untouched before reset
    for t in range(8):
        await a.issue(addr_for(set_idx=t, tag=200 + t), we=1, wdata=0x5550000 + t)
    for t in range(8):
        await a.issue(addr_for(set_idx=t, tag=200 + t), we=0)
    await a.drain()
    c.sb.report()
