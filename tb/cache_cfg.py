"""Test configuration object (passed through the UVM ConfigDB).

Holds the DUT geometry, the constrained-random biasing knobs, and a shared id
pool + reference model so the active (driver) and passive (monitor/scoreboard)
sides of the environment agree.
"""

from cocotb.queue import Queue
from ref_model import RefModel

# geometry mirrors nb_dcache_pkg defaults
LINE_BYTES = 64
WORD_BYTES = 8
WORDS_PER_LINE = LINE_BYTES // WORD_BYTES
NUM_SETS = 64
SET_SHIFT = 6
DATA_WIDTH = 64


class CacheConfig:
    def __init__(self):
        # geometry
        self.line_bytes = LINE_BYTES
        self.word_bytes = WORD_BYTES
        self.words_per_line = WORDS_PER_LINE
        self.num_sets = NUM_SETS
        self.data_width = DATA_WIDTH
        # constrained-random biasing
        self.sets = [0, 1, 2]
        self.tags = list(range(8))
        self.store_prob = 0.5
        self.strb_choices = [0xFF, 0x0F, 0xF0, 0x01, 0xAA, 0x3C]
        self.n_items = 400
        self.seed = 1
        self.axi_max_latency = 16
        # shared infrastructure
        self.n_ids = 16
        self.free_ids = Queue()
        for i in range(self.n_ids):
            self.free_ids.put_nowait(i)
        self.ref = RefModel(WORD_BYTES)

    def addr_for(self, set_idx, tag, word=0):
        return (tag << (SET_SHIFT + 6)) | (set_idx << SET_SHIFT) \
            | (word * self.word_bytes)
