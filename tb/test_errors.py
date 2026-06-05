"""AXI error-propagation tests.

A load that misses to a line whose fill returns SLVERR must report cpu_rsp_err;
unrelated lines must still succeed. The perf_bus_errors counter must move.
"""

import cocotb
from cocotb.triggers import RisingEdge

from tb_common import setup, addr_for, line_base


async def load_wait(c, rid, addr):
    before = c.mon.count
    await c.drv.send(rid, addr, we=0)
    while c.mon.count <= before:
        await RisingEdge(c.drv.dut.clk)
    return c.mon.responses[rid][-1]


@cocotb.test()
async def test_read_error_propagates(dut):
    c = await setup(dut, seed=51)
    ERR = addr_for(set_idx=3, tag=9)
    c.slave.err_read_lines.add(line_base(ERR))

    rsp = await load_wait(c, 1, ERR)
    assert rsp.err == 1, f"expected error on faulting line, got err={rsp.err}"

    # an unrelated line still works and is error-free
    OK = addr_for(set_idx=4, tag=2)
    rsp2 = await load_wait(c, 2, OK)
    assert rsp2.err == 0 and rsp2.rdata == 0

    # the design exposes a bus-error perf counter
    assert int(dut.perf_bus_errors.value) >= 1


@cocotb.test()
async def test_error_then_clean_refill(dut):
    """After a faulting fill, clearing the fault and re-accessing the line (a new
    miss, since the errored line is not cached) succeeds."""
    c = await setup(dut, seed=52)
    A = addr_for(set_idx=6, tag=1)
    c.slave.err_read_lines.add(line_base(A))
    r1 = await load_wait(c, 1, A)
    assert r1.err == 1

    c.slave.err_read_lines.discard(line_base(A))
    c.slave.write_word(A, 0x1234567899999999, 0xFF)   # give memory a value
    r2 = await load_wait(c, 2, A)
    assert r2.err == 0 and r2.rdata == 0x1234567899999999
