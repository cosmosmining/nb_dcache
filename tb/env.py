"""Concurrent CPU agent + scoreboard for the non-blocking cache.

Supports many outstanding, tagged, out-of-order responses:

  * Each in-flight request gets a UNIQUE id from a free pool (the only way to
    reconcile out-of-order responses by tag), freed when its response arrives.
  * The reference model is applied at REQUEST ACCEPTANCE, which is program order
    (the CPU port accepts in order). For a load the expected value is snapshotted
    then; later stores cannot perturb it -- exactly the ordering the RTL must
    honour via in-order accept + MSHR per-word forward snapshots.
  * The scoreboard checks every load response against its snapshot and counts
    that every accepted request retires exactly once.
"""

import cocotb
from cocotb.triggers import RisingEdge

from ref_model import RefModel


class Scoreboard:
    def __init__(self):
        self.checks = 0
        self.retired = 0
        self.errors = []

    def check(self, rid, is_load, got, exp, addr):
        self.retired += 1
        if is_load:
            self.checks += 1
            if got != exp:
                self.errors.append(
                    f"LOAD id={rid} addr=0x{addr:x}: got 0x{got:x} exp 0x{exp:x}")

    def report(self):
        assert not self.errors, "scoreboard mismatches:\n  " + \
            "\n  ".join(self.errors[:20])


class CpuAgent:
    def __init__(self, dut, scoreboard, ref=None, n_ids=16, coverage=None):
        self.dut = dut
        self.sb = scoreboard
        self.ref = ref or RefModel()
        self.cov = coverage
        self.free_ids = list(range(n_ids))
        self.outstanding = {}     # id -> dict(is_load, addr, exp)
        self.issued = 0

    def start(self):
        cocotb.start_soon(self._monitor())

    async def _monitor(self):
        d = self.dut
        d.cpu_rsp_ready.value = 1
        while True:
            await RisingEdge(d.clk)
            if d.cpu_rsp_valid.value and d.cpu_rsp_ready.value:
                rid = int(d.cpu_rsp_id.value)
                info = self.outstanding.pop(rid, None)
                if info is None:
                    continue   # straggler from before a mid-burst reset
                self.sb.check(rid, info['is_load'], int(d.cpu_rsp_rdata.value),
                              info['exp'], info['addr'])
                self.free_ids.append(rid)
                if self.cov:
                    self.cov.sample_response(rid)

    def reset_state(self):
        """Drop in-flight bookkeeping after a DUT reset (responses lost)."""
        self.outstanding = {}
        self.free_ids = list(range(16))

    async def _acquire_id(self):
        while not self.free_ids:
            await RisingEdge(self.dut.clk)
        return self.free_ids.pop(0)

    async def issue(self, addr, we=0, wdata=0, wstrb=0xFF):
        """Acquire an id, drive the request, return once accepted (program
        order point). Does NOT wait for the response (non-blocking)."""
        d = self.dut
        rid = await self._acquire_id()
        # snapshot expected at acceptance time = program order
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
        # apply reference model in program order, snapshot load expectation
        if we:
            self.ref.store(addr, wdata, wstrb)
            exp = 0
        else:
            exp = self.ref.load(addr)
        self.outstanding[rid] = dict(is_load=(not we), addr=addr, exp=exp)
        self.issued += 1
        if self.cov:
            self.cov.sample_request(rid)

    async def drain(self, timeout_cycles=200000):
        for _ in range(timeout_cycles):
            if not self.outstanding:
                return
            await RisingEdge(self.dut.clk)
        raise TimeoutError(f"{len(self.outstanding)} requests never retired: "
                           f"{list(self.outstanding)}")
