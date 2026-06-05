"""Constrained-random stress: the comprehensive correctness check.

Biased to a small number of sets to maximize conflicts (evictions during
refills, WAR re-access, replay-queue pressure), mixed loads/stores, many
outstanding requests with out-of-order AXI responses. Checked end-to-end
against the architectural reference model.
"""

import random
import cocotb

from tb_common import setup_nb, addr_for, WORD_BYTES


async def _run_stream(dut, seed, n_ops, sets, tags, max_latency, store_p=0.5):
    c = await setup_nb(dut, seed=seed, max_latency=max_latency)
    a = c.agent
    rng = random.Random(seed)
    for _ in range(n_ops):
        s = rng.choice(sets)
        t = rng.choice(tags)
        w = rng.randrange(8)
        addr = addr_for(s, t, w)
        if rng.random() < store_p:
            await a.issue(addr, we=1, wdata=rng.getrandbits(64),
                          wstrb=rng.choice([0xFF, 0x0F, 0xF0, 0x01, 0xAA]))
        else:
            await a.issue(addr, we=0)
    await a.drain()
    c.sb.report()
    assert c.sb.checks > 0


@cocotb.test()
async def test_random_hot_sets(dut):
    # heavy conflict: 3 sets, 8 tags (2x associativity) -> constant eviction
    await _run_stream(dut, seed=101, n_ops=1500, sets=[0, 1, 2],
                      tags=list(range(8)), max_latency=16)


@cocotb.test()
async def test_random_single_set(dut):
    # all pressure on ONE set: maximal pLRU / eviction / lock churn
    await _run_stream(dut, seed=202, n_ops=1200, sets=[5],
                      tags=list(range(8)), max_latency=24)


@cocotb.test()
async def test_random_store_heavy(dut):
    await _run_stream(dut, seed=303, n_ops=1200, sets=[3, 4],
                      tags=list(range(6)), max_latency=20, store_p=0.8)


@cocotb.test()
async def test_random_load_heavy_low_latency(dut):
    await _run_stream(dut, seed=404, n_ops=1500, sets=[6, 7],
                      tags=list(range(10)), max_latency=4, store_p=0.2)
