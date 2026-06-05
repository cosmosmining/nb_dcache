"""CPU-side driver and response monitor for nb_dcache.

The CPU interface is a tagged valid/ready load-store port. The driver issues
requests (honouring back-pressure) and the monitor collects responses keyed by
their tag, so tests can issue many outstanding requests and reconcile the
out-of-order responses by id.
"""

import cocotb
from cocotb.triggers import RisingEdge


class CpuResponse:
    __slots__ = ("id", "rdata", "err")

    def __init__(self, rid, rdata, err):
        self.id = rid
        self.rdata = rdata
        self.err = err


class CpuDriver:
    def __init__(self, dut):
        self.dut = dut

    async def reset_inputs(self):
        d = self.dut
        d.cpu_req_valid.value = 0
        d.cpu_req_id.value = 0
        d.cpu_req_addr.value = 0
        d.cpu_req_we.value = 0
        d.cpu_req_wdata.value = 0
        d.cpu_req_wstrb.value = 0
        d.cpu_rsp_ready.value = 1
        d.maint_req_valid.value = 0
        d.maint_req_flush.value = 0

    async def send(self, rid, addr, we=0, wdata=0, wstrb=0):
        """Drive one request and block until it is accepted (valid&ready)."""
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


class CpuMonitor:
    """Collects responses into self.responses[id] (list, for repeated ids)."""

    def __init__(self, dut):
        self.dut = dut
        self.responses = {}
        self.count = 0

    def start(self):
        cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        d.cpu_rsp_ready.value = 1
        while True:
            await RisingEdge(d.clk)
            if d.cpu_rsp_valid.value and d.cpu_rsp_ready.value:
                rid = int(d.cpu_rsp_id.value)
                rsp = CpuResponse(rid, int(d.cpu_rsp_rdata.value),
                                  int(d.cpu_rsp_err.value))
                self.responses.setdefault(rid, []).append(rsp)
                self.count += 1

    async def wait_for(self, total, timeout_cycles=20000):
        """Block until `total` responses have been observed."""
        for _ in range(timeout_cycles):
            if self.count >= total:
                return
            await RisingEdge(self.dut.clk)
        raise TimeoutError(
            f"only saw {self.count}/{total} responses before timeout")
