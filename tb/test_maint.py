"""Cache-maintenance tests: invalidate-all and flush-all.

  * INVALIDATE-ALL drops dirty data: a re-read returns memory (the store is lost).
  * FLUSH-ALL writes dirty lines back first: memory holds the store, and a
    re-read (now a miss) returns it.
"""

import cocotb
from cocotb.triggers import RisingEdge

from tb_common import setup, addr_for, line_base


async def run_maint(dut, flush):
    dut.maint_req_flush.value = 1 if flush else 0
    dut.maint_req_valid.value = 1
    while not dut.maint_busy.value:        # waits for quiescence
        await RisingEdge(dut.clk)
    dut.maint_req_valid.value = 0
    while not dut.maint_done.value:
        await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def load_wait(c, rid, addr):
    before = c.mon.count
    await c.drv.send(rid, addr, we=0)
    while c.mon.count <= before:
        await RisingEdge(c.drv.dut.clk)
    return c.mon.responses[rid][-1]


async def store_wait(c, rid, addr, data):
    before = c.mon.count
    await c.drv.send(rid, addr, we=1, wdata=data, wstrb=0xFF)
    while c.mon.count <= before:
        await RisingEdge(c.drv.dut.clk)


@cocotb.test()
async def test_invalidate_all_drops_dirty(dut):
    c = await setup(dut, seed=61)
    A = addr_for(set_idx=2, tag=3)
    await store_wait(c, 1, A, 0xAAAA5555AAAA5555)   # dirty, cached
    await run_maint(dut, flush=False)               # invalidate (no write-back)
    # store was discarded -> memory still 0, re-read misses and returns 0
    r = await load_wait(c, 2, A)
    assert r.rdata == 0, f"invalidate should drop dirty data, got 0x{r.rdata:x}"
    assert c.slave.read_word(A) == 0


@cocotb.test()
async def test_flush_all_writes_back(dut):
    c = await setup(dut, seed=62)
    A = addr_for(set_idx=4, tag=7)
    B = addr_for(set_idx=9, tag=1)
    await store_wait(c, 1, A, 0x1122334455667788)
    await store_wait(c, 2, B, 0x99AABBCCDDEEFF00)
    await run_maint(dut, flush=True)                # write back + invalidate
    # memory now holds the stores
    assert c.slave.read_word(A) == 0x1122334455667788
    assert c.slave.read_word(B) == 0x99AABBCCDDEEFF00
    # re-read (miss) returns the flushed data
    assert (await load_wait(c, 3, A)).rdata == 0x1122334455667788
    assert (await load_wait(c, 4, B)).rdata == 0x99AABBCCDDEEFF00


@cocotb.test()
async def test_flush_then_clean_lines_no_extra_writeback(dut):
    """After a flush everything is clean; a second flush writes nothing back."""
    c = await setup(dut, seed=63)
    A = addr_for(set_idx=5, tag=2)
    await store_wait(c, 1, A, 0xDEADBEEF0BADF00D)
    await run_maint(dut, flush=True)
    wb_before = int(dut.perf_writebacks.value)
    await run_maint(dut, flush=True)                # all clean/invalid now
    assert int(dut.perf_writebacks.value) == wb_before
