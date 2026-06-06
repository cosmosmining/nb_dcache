"""Pipelined-lookup throughput test.

Streams back-to-back hits at the CPU port (valid held high, id rotated each
accept) and checks the cache sustains ~1 accept/cycle — the point of the
S1->S2 pipelined lookup. Also re-checks data correctness on a sample.
"""

import cocotb
from cocotb.triggers import RisingEdge

from tb_common import setup_nb, addr_for, WORD_BYTES


@cocotb.test()
async def test_hit_throughput_1_per_cycle(dut):
    c = await setup_nb(dut, seed=91, max_latency=6)
    a = c.agent

    # warm one line (8 words) so subsequent loads all hit
    base = addr_for(set_idx=1, tag=1)
    for w in range(8):
        await a.issue(base + w * WORD_BYTES, we=1, wdata=0xAB00 + w)
    await a.drain()

    # stream loads to the warm line: hold valid, rotate id on every accept
    N = 64
    accepts = 0
    cycles = 0
    rid = 0
    dut.cpu_req_we.value = 0
    dut.cpu_req_wdata.value = 0
    dut.cpu_req_wstrb.value = 0
    dut.cpu_req_addr.value = base
    dut.cpu_req_id.value = rid
    dut.cpu_req_valid.value = 1
    while accepts < N:
        await RisingEdge(dut.clk)
        cycles += 1
        if dut.cpu_req_ready.value:
            accepts += 1
            rid = (rid + 1) & 0xF
            dut.cpu_req_id.value = rid
            dut.cpu_req_addr.value = base + ((accepts % 8) * WORD_BYTES)
        assert cycles < 4 * N, f"throughput collapsed: {accepts} accepts in {cycles} cycles"
    dut.cpu_req_valid.value = 0

    # 1/cycle steady state (allow a little pipeline warm-up slack)
    assert cycles <= N + 8, \
        f"expected ~1 accept/cycle, got {accepts} accepts in {cycles} cycles"
    cocotb.log.info("hit throughput: %d accepts in %d cycles", accepts, cycles)

    # data correctness spot-check after the storm
    await RisingEdge(dut.clk)
    for w in range(8):
        await a.issue(base + w * WORD_BYTES, we=0)
    await a.drain()
    c.sb.report()
