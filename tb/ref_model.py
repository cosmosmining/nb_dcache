"""Architectural reference model for the nb_dcache.

This is a *memory-semantics* model, NOT a microarchitectural mirror of the RTL.
It knows nothing about lines, ways, pLRU, MSHRs or AXI. It models a single flat
byte-addressable memory observed through the CPU port:

  * the CPU port accepts requests in program order (one at a time via
    valid/ready), so the model applies operations in acceptance order;
  * a load returns the most recent byte values written to that word, or 0 if the
    word was never written (cold memory is zero, matching the AXI slave);
  * responses may come back out of order, but correctness is checked by *id*:
    the expected load value is captured at issue time.

The scoreboard compares each DUT load response against the value this model
predicted for that request id.
"""


class RefModel:
    def __init__(self, word_bytes=8):
        self.WB = word_bytes
        self.MASK = (1 << (word_bytes * 8)) - 1
        self.mem = {}

    def _base(self, addr):
        return addr & ~(self.WB - 1)

    def load(self, addr):
        """Return the architectural value of the word containing addr."""
        return self.mem.get(self._base(addr), 0)

    def store(self, addr, data, strb):
        base = self._base(addr)
        cur = self.mem.get(base, 0)
        for b in range(self.WB):
            if (strb >> b) & 1:
                mask = 0xFF << (b * 8)
                cur = (cur & ~mask) | (data & mask)
        self.mem[base] = cur & self.MASK
