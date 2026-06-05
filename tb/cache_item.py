"""UVM sequence item for the nb_dcache CPU load/store interface."""

from pyuvm import uvm_sequence_item


class CacheItem(uvm_sequence_item):
    """One CPU request (and, after the test, its observed response).

    Request fields are randomized by the sequences; `rid` is assigned by the
    driver from the id pool; response fields are filled by the monitor/scoreboard
    for reporting only.
    """

    def __init__(self, name="CacheItem"):
        super().__init__(name)
        # request
        self.addr = 0
        self.we = 0
        self.wdata = 0
        self.wstrb = 0xFF
        # bookkeeping
        self.rid = None
        # observed response
        self.rdata = None

    def randomize(self, rng, cfg):
        """Constrained-random fill biased per the test configuration."""
        s = rng.choice(cfg.sets)
        t = rng.choice(cfg.tags)
        w = rng.randrange(cfg.words_per_line)
        self.addr = cfg.addr_for(s, t, w)
        if rng.random() < cfg.store_prob:
            self.we = 1
            self.wdata = rng.getrandbits(cfg.data_width)
            self.wstrb = rng.choice(cfg.strb_choices)
        else:
            self.we = 0
            self.wdata = 0
            self.wstrb = 0
        return self

    def __str__(self):
        kind = "ST" if self.we else "LD"
        rid = "-" if self.rid is None else self.rid
        return (f"{kind} id={rid} addr=0x{self.addr:08x} "
                f"wdata=0x{self.wdata:016x} wstrb=0x{self.wstrb:02x}")
