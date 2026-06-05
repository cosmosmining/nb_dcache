"""Stage-3 tests: miss-under-miss + secondary-miss merging.

Covers the named hazards:
  * back-to-back same-line misses merged into one MSHR (per-word sub-entries)
  * store-then-load-same-address before the fill returns (store-to-load
    forwarding in the fill window)
  * write merging into an in-flight MSHR
"""

import random
import cocotb

from tb_common import setup_nb, addr_for, WORD_BYTES


@cocotb.test()
async def test_secondary_miss_merge(dut):
    """Many accesses to the SAME cold line while its fill is outstanding must
    merge into a single MSHR and all retire correctly."""
    c = await setup_nb(dut, seed=21, max_latency=40)
    a = c.agent
    base = addr_for(set_idx=9, tag=4)
    # first access -> primary miss; the rest (same line, different words) merge
    for w in range(8):
        await a.issue(base + w * WORD_BYTES, we=0)
    await a.drain()
    c.sb.report()


@cocotb.test()
async def test_store_to_load_forward_in_fill(dut):
    """Store to a cold line (primary miss) then load the SAME word before the
    fill returns: the load must forward the merged store data."""
    c = await setup_nb(dut, seed=22, max_latency=50)
    a = c.agent
    A = addr_for(set_idx=10, tag=7)
    await a.issue(A, we=1, wdata=0xCAFED00DCAFED00D)   # primary miss (store)
    await a.issue(A, we=0)                              # merged load -> forward
    # also a load to a different word of the same line -> reads memory (0)
    await a.issue(A + WORD_BYTES, we=0)
    await a.drain()
    c.sb.report()


@cocotb.test()
async def test_write_merge_into_mshr(dut):
    """Multiple stores merge into one in-flight MSHR; a trailing load sees the
    merged result; words not stored read memory."""
    c = await setup_nb(dut, seed=23, max_latency=40)
    a = c.agent
    base = addr_for(set_idx=12, tag=1)
    await a.issue(base + 0 * WORD_BYTES, we=1, wdata=0x1111111111111111)  # primary
    await a.issue(base + 1 * WORD_BYTES, we=1, wdata=0x2222222222222222)  # merge
    await a.issue(base + 2 * WORD_BYTES, we=1, wdata=0x3333333333333333)  # merge
    await a.issue(base + 0 * WORD_BYTES, we=0)   # load merged word -> forward
    await a.issue(base + 3 * WORD_BYTES, we=0)   # never stored -> memory (0)
    await a.drain()
    c.sb.report()


@cocotb.test()
async def test_partial_strb_merge(dut):
    """Byte-granular store merging into an in-flight MSHR, then a forwarded
    load combining stored and memory bytes."""
    c = await setup_nb(dut, seed=24, max_latency=40)
    a = c.agent
    A = addr_for(set_idx=13, tag=6)
    # store only the low 4 bytes
    await a.issue(A, we=1, wdata=0x00000000DEADBEEF, wstrb=0x0F)
    # forwarded load: low 4 bytes = stored, high 4 = memory (0)
    await a.issue(A, we=0)
    await a.drain()
    c.sb.report()


@cocotb.test()
async def test_same_line_mixed_stream(dut):
    """Randomized mixed loads/stores all to one cold line during the fill."""
    c = await setup_nb(dut, seed=25, max_latency=60)
    a = c.agent
    rng = random.Random(7)
    base = addr_for(set_idx=14, tag=3)
    for _ in range(12):
        w = rng.randrange(8)
        if rng.random() < 0.5:
            await a.issue(base + w * WORD_BYTES, we=1, wdata=rng.getrandbits(64))
        else:
            await a.issue(base + w * WORD_BYTES, we=0)
    await a.drain()
    c.sb.report()
