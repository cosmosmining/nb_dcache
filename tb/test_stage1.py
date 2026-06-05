"""Stage-1 directed tests: blocking write-back / write-allocate cache.

Validates the baseline before any non-blocking machinery exists:
  * cold load miss returns zero-backed memory
  * store hit / load hit
  * write-allocate on store miss
  * dirty eviction + write-back (data survives a conflict eviction)
  * a randomized mixed load/store stream checked against the reference model
"""

import random
import cocotb
from cocotb.triggers import RisingEdge

from tb_common import setup, addr_for, WORD_BYTES, WAYS


async def do_load(c, rid, addr):
    """Issue a load, return after its response is observed; check vs ref."""
    expected = c.ref.load(addr)
    before = c.mon.count
    await c.drv.send(rid, addr, we=0)
    while c.mon.count <= before:
        await RisingEdge(c.drv.dut.clk)
    rsp = c.mon.responses[rid][-1]
    assert rsp.rdata == expected, \
        f"load id={rid} addr=0x{addr:x}: got 0x{rsp.rdata:x} exp 0x{expected:x}"
    return rsp.rdata


async def do_store(c, rid, addr, data, strb=0xFF):
    before = c.mon.count
    await c.drv.send(rid, addr, we=1, wdata=data, wstrb=strb)
    c.ref.store(addr, data, strb)
    while c.mon.count <= before:
        await RisingEdge(c.drv.dut.clk)


@cocotb.test()
async def test_cold_load_zero(dut):
    c = await setup(dut, seed=1)
    for i in range(4):
        await do_load(c, i, addr_for(set_idx=i, tag=i))


@cocotb.test()
async def test_store_then_load_hit(dut):
    c = await setup(dut, seed=2)
    a = addr_for(set_idx=3, tag=5)
    await do_store(c, 1, a, 0xDEADBEEFCAFEBABE)
    v = await do_load(c, 2, a)
    assert v == 0xDEADBEEFCAFEBABE


@cocotb.test()
async def test_write_allocate(dut):
    """Store to a cold line (miss) then read neighbouring words in the line."""
    c = await setup(dut, seed=3)
    base = addr_for(set_idx=7, tag=9)
    await do_store(c, 1, base + 0 * WORD_BYTES, 0x1111111111111111)
    await do_store(c, 2, base + 1 * WORD_BYTES, 0x2222222222222222)
    assert (await do_load(c, 3, base + 0 * WORD_BYTES)) == 0x1111111111111111
    assert (await do_load(c, 4, base + 1 * WORD_BYTES)) == 0x2222222222222222


@cocotb.test()
async def test_dirty_eviction_writeback(dut):
    """Fill all WAYS+1 tags of one set with dirty stores, forcing eviction of a
    dirty line; then read every line back to confirm the evicted data round-
    tripped through main memory."""
    c = await setup(dut, seed=4)
    s = 11
    tags = list(range(WAYS + 2))          # more tags than ways -> evictions
    for i, t in enumerate(tags):
        await do_store(c, i & 0xF, addr_for(s, t), 0xA0000000_00000000 + t)
    # read them all back (some served from memory after eviction)
    for i, t in enumerate(tags):
        v = await do_load(c, (8 + i) & 0xF, addr_for(s, t))
        assert v == 0xA0000000_00000000 + t, f"tag {t}: 0x{v:x}"


@cocotb.test()
async def test_random_mixed(dut):
    """Randomized loads/stores biased to a few sets; checked vs reference."""
    c = await setup(dut, seed=5)
    rng = random.Random(123)
    sets = [2, 3, 4]                       # bias to force conflicts/evictions
    tags = list(range(8))
    rid = 0
    for _ in range(200):
        s = rng.choice(sets)
        t = rng.choice(tags)
        w = rng.randrange(8)
        a = addr_for(s, t, w)
        if rng.random() < 0.5:
            await do_store(c, rid & 0xF, a, rng.getrandbits(64))
        else:
            await do_load(c, rid & 0xF, a)
        rid += 1
