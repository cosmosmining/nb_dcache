"""Functional coverage for nb_dcache.

Bins required by the spec:
  * MSHR occupancy 0..4
  * merge events
  * replays
  * dirty vs clean evictions
  * observed response reordering
  * one bin per hazard case (MSHR full / no free way / write-back conflict /
    merge-window miss)

Most events are tapped from the synthesizable debug bus (dbg_*); response
reordering is derived in the agent by comparing retirement order to issue order.
"""

import cocotb
from cocotb.triggers import RisingEdge


class Coverage:
    HAZARDS = ["mshr_full", "no_free_way", "wb_conflict", "merge_window"]

    def __init__(self, dut):
        self.dut = dut
        self.occ_bins = set()
        self.merge = 0
        self.replay = 0
        self.evict_dirty = 0
        self.evict_clean = 0
        self.hazard = {h: 0 for h in self.HAZARDS}
        self.reorder = 0
        self._seq = {}          # id -> issue sequence number (while outstanding)
        self._next = 0

    def start(self):
        cocotb.start_soon(self._sample())

    async def _sample(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk)
            if not d.rst_n.value:
                continue
            self.occ_bins.add(int(d.dbg_mshr_occupancy.value))
            self.merge += int(d.dbg_merge.value)
            self.replay += int(d.dbg_replay.value)
            self.evict_dirty += int(d.dbg_evict_dirty.value)
            self.evict_clean += int(d.dbg_evict_clean.value)
            h = int(d.dbg_hazard.value)
            for b, name in enumerate(self.HAZARDS):
                if (h >> b) & 1:
                    self.hazard[name] += 1

    # ----- called by the agent --------------------------------------------
    def sample_request(self, rid):
        self._seq[rid] = self._next
        self._next += 1

    def sample_response(self, rid):
        s = self._seq.pop(rid, None)
        if s is None:
            return
        # reordering: an older outstanding request exists -> this retired early
        if self._seq and min(self._seq.values()) < s:
            self.reorder += 1

    # ----- reporting -------------------------------------------------------
    def summary(self):
        return {
            "occupancy_bins": sorted(self.occ_bins),
            "merge": self.merge,
            "replay": self.replay,
            "evict_dirty": self.evict_dirty,
            "evict_clean": self.evict_clean,
            "reorder": self.reorder,
            "hazard": dict(self.hazard),
        }

    def assert_closed(self, need_occ=(0, 1, 2, 3, 4)):
        """Assert every required bin was hit at least once (coverage closure)."""
        missing = []
        for o in need_occ:
            if o not in self.occ_bins:
                missing.append(f"occupancy={o}")
        for name, cnt in [("merge", self.merge), ("replay", self.replay),
                          ("evict_dirty", self.evict_dirty),
                          ("evict_clean", self.evict_clean),
                          ("reorder", self.reorder)]:
            if cnt == 0:
                missing.append(name)
        for h, cnt in self.hazard.items():
            if cnt == 0:
                missing.append(f"hazard:{h}")
        assert not missing, f"coverage NOT closed, missing bins: {missing}\n" \
                            f"summary={self.summary()}"
