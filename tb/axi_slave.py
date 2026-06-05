"""Reusable AXI4 slave memory model for the nb_dcache testbench.

Behaviour intentionally stresses the DUT:

  * Randomized response latency on both R and B.
  * OUT-OF-ORDER completion across IDs: whole read bursts (and write responses)
    may return in a different order than issued. AXI4 forbids *beat-level*
    interleaving of read data, so each burst's beats are driven contiguously and
    in order; reordering happens at transaction granularity by randomly picking
    among the transactions whose latency has elapsed.
  * Backing store is a flat byte-addressable memory defaulting to 0, matching the
    architectural reference model's initial state.

The model drives the slave-side signals (ARREADY, R*, AWREADY, WREADY, B*) and
samples the master-side signals on the DUT (m_* names).
"""

import random
from cocotb.triggers import RisingEdge


class AxiSlaveMemory:
    def __init__(self, dut, data_width=64, max_latency=12, seed=None):
        self.dut = dut
        self.DW = data_width
        self.WB = data_width // 8
        self.max_latency = max_latency
        self.mem = {}                      # word-aligned byte addr -> int
        self.rng = random.Random(seed)
        self.LB = 64                       # bytes per cache line (for masking)
        self.err_read_lines = set()        # line-base addrs that return SLVERR on R
        self.err_write_lines = set()       # line-base addrs that return SLVERR on B
        self._pending_reads = []           # list of dicts: id, addr, len, delay
        self._pending_writes = []          # list of dicts: id, delay
        self._aw_q = []                    # accepted AW awaiting W data
        self._wbeats = []                  # collected W beats for current burst

    # ----- backing-store helpers ------------------------------------------
    def read_word(self, addr):
        return self.mem.get(addr & ~(self.WB - 1), 0)

    def write_word(self, addr, data, strb):
        base = addr & ~(self.WB - 1)
        cur = self.mem.get(base, 0)
        for b in range(self.WB):
            if (strb >> b) & 1:
                mask = 0xFF << (b * 8)
                cur = (cur & ~mask) | (data & mask)
        self.mem[base] = cur & ((1 << self.DW) - 1)

    # ----- public: launch all channel servers ------------------------------
    def start(self):
        """Launch all channel servers; returns the spawned tasks so callers can
        tear them down (needed when several tests share one simulation)."""
        import cocotb
        self.tasks = [
            cocotb.start_soon(self._ar_server()),
            cocotb.start_soon(self._r_server()),
            cocotb.start_soon(self._aw_server()),
            cocotb.start_soon(self._w_server()),
            cocotb.start_soon(self._b_server()),
        ]
        return self.tasks

    # ----- read address ----------------------------------------------------
    async def _ar_server(self):
        d = self.dut
        d.m_arready.value = 1
        while True:
            await RisingEdge(d.clk)
            if d.m_arvalid.value and d.m_arready.value:
                self._pending_reads.append(dict(
                    id=int(d.m_arid.value),
                    addr=int(d.m_araddr.value),
                    len=int(d.m_arlen.value),
                    delay=self.rng.randint(0, self.max_latency)))

    # ----- read data (OoO across IDs) -------------------------------------
    async def _r_server(self):
        d = self.dut
        d.m_rvalid.value = 0
        while True:
            await RisingEdge(d.clk)
            for p in self._pending_reads:
                p['delay'] -= 1
            ready = [p for p in self._pending_reads if p['delay'] < 0]
            if not ready:
                continue
            job = self.rng.choice(ready)
            self._pending_reads.remove(job)
            base = job['addr'] & ~(self.WB - 1)
            resp = 2 if (base & ~(self.LB - 1)) in self.err_read_lines else 0  # 2=SLVERR
            for beat in range(job['len'] + 1):
                a = base + beat * self.WB
                d.m_rvalid.value = 1
                d.m_rid.value = job['id']
                d.m_rdata.value = self.read_word(a)
                d.m_rresp.value = resp
                d.m_rlast.value = 1 if beat == job['len'] else 0
                await RisingEdge(d.clk)
                while not d.m_rready.value:
                    await RisingEdge(d.clk)
            d.m_rvalid.value = 0
            d.m_rlast.value = 0

    # ----- write address ---------------------------------------------------
    async def _aw_server(self):
        d = self.dut
        d.m_awready.value = 1
        while True:
            await RisingEdge(d.clk)
            if d.m_awvalid.value and d.m_awready.value:
                self._aw_q.append(dict(
                    id=int(d.m_awid.value),
                    addr=int(d.m_awaddr.value),
                    len=int(d.m_awlen.value)))

    # ----- write data ------------------------------------------------------
    async def _w_server(self):
        d = self.dut
        d.m_wready.value = 1
        while True:
            await RisingEdge(d.clk)
            if d.m_wvalid.value and d.m_wready.value:
                self._wbeats.append((int(d.m_wdata.value), int(d.m_wstrb.value)))
                if d.m_wlast.value:
                    # W data follows AW order in AXI4: pair with oldest AW.
                    aw = self._aw_q.pop(0)
                    base = aw['addr'] & ~(self.WB - 1)
                    for i, (wd, ws) in enumerate(self._wbeats):
                        self.write_word(base + i * self.WB, wd, ws)
                    self._wbeats = []
                    self._pending_writes.append(dict(
                        id=aw['id'], base=base,
                        delay=self.rng.randint(0, self.max_latency)))

    # ----- write response (OoO across IDs) --------------------------------
    async def _b_server(self):
        d = self.dut
        d.m_bvalid.value = 0
        while True:
            await RisingEdge(d.clk)
            for p in self._pending_writes:
                p['delay'] -= 1
            ready = [p for p in self._pending_writes if p['delay'] < 0]
            if not ready:
                continue
            job = self.rng.choice(ready)
            self._pending_writes.remove(job)
            d.m_bvalid.value = 1
            d.m_bid.value = job['id']
            d.m_bresp.value = 2 if (job['base'] & ~(self.LB - 1)) \
                in self.err_write_lines else 0
            await RisingEdge(d.clk)
            while not d.m_bready.value:
                await RisingEdge(d.clk)
            d.m_bvalid.value = 0
