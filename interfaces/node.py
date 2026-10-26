import logging
import threading
import time

from interfaces.bus import Publisher, Subscriber


class Node:
    name = "node"
    rate_hz = 30.0
    subscribe = []

    def __init__(self, bus_cfg=None, **params):
        self.params = params
        self.bus_cfg = bus_cfg
        self.log = logging.getLogger(self.name)
        self.pub = None
        self.sub = None
        self._stop = threading.Event()
        self._thread = None

    def setup(self):
        pass

    def step(self):
        pass

    def teardown(self):
        pass

    def publish(self, topic, msg):
        self.pub.publish(topic, msg)

    def start(self):
        self.pub = Publisher(self.bus_cfg)
        if self.subscribe:
            self.sub = Subscriber(self.subscribe, self.bus_cfg)
        self._thread = threading.Thread(target=self._run, name=self.name, daemon=True)
        self._thread.start()
        return self

    def _run(self):
        self.setup()
        period = 1.0 / self.rate_hz
        next_t = time.perf_counter()
        try:
            while not self._stop.is_set():
                self.step()
                next_t += period
                delay = next_t - time.perf_counter()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    next_t = time.perf_counter()
        except Exception:
            self.log.exception(f"{self.name} crashed")
        finally:
            self.teardown()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for s in (self.pub, self.sub):
            if s is not None:
                s.close()

    @property
    def alive(self):
        return self._thread is not None and self._thread.is_alive()
