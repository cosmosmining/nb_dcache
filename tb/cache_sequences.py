"""UVM sequence library for nb_dcache.

A constrained-random base sequence plus directed sequences for each named
hazard. Sequences pull the test configuration from the ConfigDB so a test can
retune biasing without editing the sequence.
"""

import random
from pyuvm import uvm_sequence, ConfigDB

from cache_item import CacheItem


class _Base(uvm_sequence):
    def cfg(self):
        return ConfigDB().get(None, "", "cfg")

    async def do(self, addr, we=0, wdata=0, wstrb=0xFF):
        it = CacheItem()
        it.addr, it.we, it.wdata, it.wstrb = addr, we, wdata, wstrb
        await self.start_item(it)
        await self.finish_item(it)


class RandomSeq(_Base):
    """Constrained-random mixed loads/stores biased to a few sets."""
    async def body(self):
        cfg = self.cfg()
        rng = random.Random(cfg.seed)
        for _ in range(cfg.n_items):
            it = CacheItem().randomize(rng, cfg)
            await self.start_item(it)
            await self.finish_item(it)


class HitUnderMissSeq(_Base):
    """Warm a line, then a slow miss followed by a burst of hits."""
    async def body(self):
        cfg = self.cfg()
        B = cfg.addr_for(5, 2)
        await self.do(B, we=1, wdata=0xB0B0B0B0B0B0B0B0)   # warm B (cache it)
        await self.do(cfg.addr_for(6, 3), we=0)            # slow miss A
        for _ in range(8):
            await self.do(B, we=0)                         # hits under the miss


class MergeSeq(_Base):
    """Back-to-back same-line misses + store-to-load forwarding + write merge."""
    async def body(self):
        cfg = self.cfg()
        base = cfg.addr_for(9, 4)
        # primary store miss, then forwarded load to same word, then loads
        await self.do(base, we=1, wdata=0xCAFED00DCAFED00D)
        await self.do(base, we=0)                          # store-to-load forward
        for w in range(cfg.words_per_line):
            await self.do(base + w * cfg.word_bytes, we=(w & 1),
                          wdata=0x1000 + w)


class FullPressureSeq(_Base):
    """More distinct in-flight miss lines than MSHRs -> replay pressure."""
    async def body(self):
        cfg = self.cfg()
        for t in range(16):
            await self.do(cfg.addr_for(t % 8, 10 + t), we=(t & 1),
                          wdata=0xDD00 + t)


class EvictDuringRefillSeq(_Base):
    """Hammer one set with > ways tags, then read everything back."""
    async def body(self):
        cfg = self.cfg()
        s = 20
        for t in range(10):
            await self.do(cfg.addr_for(s, t), we=1, wdata=0xE0000000 + t)
        for t in range(10):
            await self.do(cfg.addr_for(s, t), we=0)
