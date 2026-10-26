import json
import threading
import time
from pathlib import Path

import yaml
import zmq

from interfaces.messages import decode, encode

ROOT = Path(__file__).resolve().parents[1]
BUS_CONFIG = ROOT / "config" / "bus.yaml"


def load_bus_config(path=BUS_CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


class Broker:
    def __init__(self, cfg=None):
        self.cfg = cfg or load_bus_config()
        self.ctx = zmq.Context.instance()
        self._thread = None

    def start(self):
        bind = self.cfg.get("bind", "0.0.0.0")
        self.xsub = self.ctx.socket(zmq.XSUB)
        self.xsub.bind(f"tcp://{bind}:{self.cfg['pub_port']}")
        self.xpub = self.ctx.socket(zmq.XPUB)
        self.xpub.bind(f"tcp://{bind}:{self.cfg['sub_port']}")
        self._thread = threading.Thread(target=self._run, daemon=True, name="bus_broker")
        self._thread.start()
        return self

    def _run(self):
        try:
            zmq.proxy(self.xsub, self.xpub)
        except zmq.ContextTerminated:
            pass


class Publisher:
    def __init__(self, cfg=None):
        cfg = cfg or load_bus_config()
        self.sock = zmq.Context.instance().socket(zmq.PUB)
        self.sock.connect(f"tcp://{cfg['host']}:{cfg['pub_port']}")
        self.lock = threading.Lock()

    def publish(self, topic, msg):
        payload = json.dumps(encode(msg)).encode()
        with self.lock:
            self.sock.send_multipart([topic.encode(), payload])

    def close(self):
        self.sock.close(linger=0)


class Subscriber:
    def __init__(self, topics, cfg=None):
        cfg = cfg or load_bus_config()
        self.topics = [topics] if isinstance(topics, str) else list(topics)
        self.sock = zmq.Context.instance().socket(zmq.SUB)
        self.sock.connect(f"tcp://{cfg['host']}:{cfg['sub_port']}")
        for t in self.topics:
            self.sock.setsockopt(zmq.SUBSCRIBE, t.encode())
        self._latest = {}
        self._stamps = {}

    def recv(self, timeout_ms=None):
        while True:
            if timeout_ms is not None and not self.sock.poll(timeout_ms):
                return None
            topic, payload = self.sock.recv_multipart()
            topic = topic.decode()
            if topic in self.topics:
                break
            timeout_ms = 0
        msg = decode(json.loads(payload))
        self._latest[topic] = msg
        self._stamps[topic] = time.time()
        return topic, msg

    def drain(self):
        out = []
        while self.sock.poll(0):
            r = self.recv(0)
            if r is not None:
                out.append(r)
        return out

    def latest(self, topic, max_age=None):
        self.drain()
        if topic not in self._latest:
            return None
        if max_age is not None and time.time() - self._stamps[topic] > max_age:
            return None
        return self._latest[topic]

    def close(self):
        self.sock.close(linger=0)
