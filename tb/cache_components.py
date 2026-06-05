"""UVM components for the nb_dcache environment.

Topology:

    CacheEnv
      |- CacheAgent
      |    |- uvm_sequencer  --> CacheDriver  --(drives bus via CpuBfm)
      |    |- CacheReqMonitor (passive)  --req_ap-->
      |    \- CacheRspMonitor (passive)  --rsp_ap-->
      |- CacheScoreboard  (reference-model checks, fed by req_ap/rsp_ap)
      \- CacheCoverage    (functional coverage, fed by req_ap/rsp_ap + dbg bus)

Active/passive split: the driver only wiggles pins; the monitors reconstruct
transactions by observing the bus, so checking does not depend on the driver.
"""

import cocotb
from pyuvm import (uvm_driver, uvm_monitor, uvm_agent, uvm_scoreboard,
                   uvm_subscriber, uvm_component, uvm_env, uvm_sequencer,
                   uvm_analysis_port, ConfigDB)

from coverage import Coverage


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------
class CacheDriver(uvm_driver):
    async def run_phase(self):
        self.cfg = ConfigDB().get(self, "", "cfg")
        self.bfm = ConfigDB().get(self, "", "bfm")
        while True:
            item = await self.seq_item_port.get_next_item()
            rid = await self.cfg.free_ids.get()      # back-pressure on id pool
            item.rid = rid
            await self.bfm.drive(rid, item.addr, item.we, item.wdata, item.wstrb)
            self.seq_item_port.item_done()


# ---------------------------------------------------------------------------
# Monitors (passive bus observers)
# ---------------------------------------------------------------------------
class CacheReqMonitor(uvm_monitor):
    def build_phase(self):
        self.ap = uvm_analysis_port("ap", self)

    async def run_phase(self):
        self.bfm = ConfigDB().get(self, "", "bfm")
        while True:
            obs = await self.bfm.req_q.get()
            self.ap.write(obs)


class CacheRspMonitor(uvm_monitor):
    def build_phase(self):
        self.ap = uvm_analysis_port("ap", self)

    async def run_phase(self):
        self.bfm = ConfigDB().get(self, "", "bfm")
        self.cfg = ConfigDB().get(self, "", "cfg")
        while True:
            obs = await self.bfm.rsp_q.get()
            self.ap.write(obs)
            self.cfg.free_ids.put_nowait(obs.rid)   # recycle the id


# ---------------------------------------------------------------------------
# Agent
# ---------------------------------------------------------------------------
class CacheAgent(uvm_agent):
    def build_phase(self):
        self.seqr = uvm_sequencer("seqr", self)
        self.driver = CacheDriver("driver", self)
        self.req_mon = CacheReqMonitor("req_mon", self)
        self.rsp_mon = CacheRspMonitor("rsp_mon", self)

    def connect_phase(self):
        self.driver.seq_item_port.connect(self.seqr.seq_item_export)


# ---------------------------------------------------------------------------
# Scoreboard
# ---------------------------------------------------------------------------
class _ReqSub(uvm_subscriber):
    def write(self, item):
        self.sb.on_req(item)


class _RspSub(uvm_subscriber):
    def write(self, item):
        self.sb.on_rsp(item)


class CacheScoreboard(uvm_scoreboard):
    def build_phase(self):
        self.cfg = ConfigDB().get(self, "", "cfg")
        self.req_sub = _ReqSub("req_sub", self)
        self.rsp_sub = _RspSub("rsp_sub", self)
        self.req_sub.sb = self
        self.rsp_sub.sb = self
        self.expected = {}        # rid -> (addr, expected_value)  (loads only)
        self.errors = []
        self.checks = 0
        self.retired = 0

    # Requests arrive (in program order) strictly before their responses in sim
    # time, so applying the reference model here is correct.
    def on_req(self, o):
        if o.we:
            self.cfg.ref.store(o.addr, o.wdata, o.wstrb)
        else:
            self.expected[o.rid] = (o.addr, self.cfg.ref.load(o.addr))

    def on_rsp(self, o):
        self.retired += 1
        if o.rid in self.expected:        # load -> check data
            addr, exp = self.expected.pop(o.rid)
            self.checks += 1
            if o.rdata != exp:
                self.errors.append(
                    f"LOAD id={o.rid} addr=0x{addr:x}: got 0x{o.rdata:x} "
                    f"exp 0x{exp:x}")

    def check(self):
        assert not self.errors, "scoreboard mismatches:\n  " + \
            "\n  ".join(self.errors[:20])
        assert self.checks > 0, "no load responses were ever checked"


# ---------------------------------------------------------------------------
# Coverage collector
# ---------------------------------------------------------------------------
class _CovReqSub(uvm_subscriber):
    def write(self, item):
        self.cov.sample_request(item.rid)


class _CovRspSub(uvm_subscriber):
    def write(self, item):
        self.cov.sample_response(item.rid)


class CacheCoverage(uvm_component):
    def build_phase(self):
        self.cov = Coverage(cocotb.top)
        self.req_sub = _CovReqSub("req_sub", self)
        self.rsp_sub = _CovRspSub("rsp_sub", self)
        self.req_sub.cov = self.cov
        self.rsp_sub.cov = self.cov

    async def run_phase(self):
        self.cov.start()              # cycle-based dbg-bus sampler

    def summary(self):
        return self.cov.summary()


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
class CacheEnv(uvm_env):
    def build_phase(self):
        self.agent = CacheAgent("agent", self)
        self.sb = CacheScoreboard("sb", self)
        self.cov = CacheCoverage("cov", self)

    def connect_phase(self):
        self.agent.req_mon.ap.connect(self.sb.req_sub.analysis_export)
        self.agent.rsp_mon.ap.connect(self.sb.rsp_sub.analysis_export)
        self.agent.req_mon.ap.connect(self.cov.req_sub.analysis_export)
        self.agent.rsp_mon.ap.connect(self.cov.rsp_sub.analysis_export)
