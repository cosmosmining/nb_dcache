"""Bus-functional model for the CPU port.

Separates pin wiggling from the UVM components:
  * the DRIVER calls `drive()` to issue an accepted request;
  * the MONITORS consume `req_q` / `rsp_q`, which background samplers fill by
    passively observing the bus handshakes (so monitoring is independent of the
    driver -- the professional UVM split).
"""

import cocotb
from cocotb.queue import Queue
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


class ObservedReq:
    __slots__ = ("rid", "addr", "we", "wdata", "wstrb")

    def __init__(self, rid, addr, we, wdata, wstrb):
        self.rid, self.addr, self.we = rid, addr, we
        self.wdata, self.wstrb = wdata, wstrb


class ObservedRsp:
    __slots__ = ("rid", "rdata")

    def __init__(self, rid, rdata):
        self.rid, self.rdata = rid, rdata


class CpuBfm:
    def __init__(self, dut):
        self.dut = dut
        self.req_q = Queue()
        self.rsp_q = Queue()

    # ----- bring-up -------------------------------------------------------
    async def start_clock(self, period_ns=10):
        self._clk_task = cocotb.start_soon(
            Clock(self.dut.clk, period_ns, unit="ns").start())
        return self._clk_task

    async def reset(self, cycles=5):
        d = self.dut
        d.cpu_req_valid.value = 0
        d.cpu_req_id.value = 0
        d.cpu_req_addr.value = 0
        d.cpu_req_we.value = 0
        d.cpu_req_wdata.value = 0
        d.cpu_req_wstrb.value = 0
        d.cpu_rsp_ready.value = 1
        d.rst_n.value = 0
        await ClockCycles(d.clk, cycles)
        d.rst_n.value = 1
        await RisingEdge(d.clk)

    def start_monitors(self):
        self._mon_tasks = [
            cocotb.start_soon(self._req_sampler()),
            cocotb.start_soon(self._rsp_sampler()),
        ]
        return self._mon_tasks

    # ----- active: drive a request ---------------------------------------
    async def drive(self, rid, addr, we, wdata, wstrb):
        d = self.dut
        d.cpu_req_valid.value = 1
        d.cpu_req_id.value = rid
        d.cpu_req_addr.value = addr
        d.cpu_req_we.value = we
        d.cpu_req_wdata.value = wdata
        d.cpu_req_wstrb.value = wstrb
        await RisingEdge(d.clk)
        while not d.cpu_req_ready.value:
            await RisingEdge(d.clk)
        d.cpu_req_valid.value = 0

    # ----- passive: sample the bus ---------------------------------------
    async def _req_sampler(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk)
            if d.rst_n.value and d.cpu_req_valid.value and d.cpu_req_ready.value:
                self.req_q.put_nowait(ObservedReq(
                    int(d.cpu_req_id.value), int(d.cpu_req_addr.value),
                    int(d.cpu_req_we.value), int(d.cpu_req_wdata.value),
                    int(d.cpu_req_wstrb.value)))

    async def _rsp_sampler(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk)
            if d.rst_n.value and d.cpu_rsp_valid.value and d.cpu_rsp_ready.value:
                self.rsp_q.put_nowait(ObservedRsp(
                    int(d.cpu_rsp_id.value), int(d.cpu_rsp_rdata.value)))
