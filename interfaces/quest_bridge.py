import json
import queue
import socket
import threading

from interfaces.bus import Subscriber
from interfaces.messages import Topic, decode, encode
from interfaces.node import Node

TO_QUEST = [
    Topic.OBJECTS, Topic.ARM_STATE, Topic.ARM_PLAN, Topic.ARM_WORKSPACE, Topic.SAFETY, Topic.TASK_STATE,
    Topic.SIM_SCENE_INFO, Topic.SIM_SCENE,
]
FROM_QUEST = {Topic.HAND, Topic.OPERATOR_COMMAND, Topic.OPERATOR_TELEOP}
LATEST_ONLY = {Topic.SIM_SCENE, Topic.OBJECTS, Topic.ARM_STATE, Topic.SAFETY}
STICKY = {Topic.SIM_SCENE_INFO, Topic.ARM_WORKSPACE}


class Client:
    def __init__(self, sock, addr, log):
        self.sock = sock
        self.addr = addr
        self.log = log
        self.queue = queue.Queue(maxsize=200)
        self.alive = True
        self.writer = threading.Thread(target=self._write, daemon=True)
        self.writer.start()

    def send(self, line):
        try:
            self.queue.put_nowait(line)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            self.queue.put_nowait(line)

    def _write(self):
        while self.alive:
            try:
                line = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.sock.sendall(line)
            except OSError:
                self.close()

    def close(self):
        if self.alive:
            self.alive = False
            self.log.info(f"quest {self.addr[0]} disconnected")
            try:
                self.sock.close()
            except OSError:
                pass


class QuestBridge(Node):
    name = "quest_bridge"
    rate_hz = 120.0

    def setup(self):
        self.port = self.params.get("port", (self.bus_cfg or {}).get("quest_port", 5560))
        self.sub = Subscriber(TO_QUEST, self.bus_cfg, allow_ground_truth=True)
        self.clients = []
        self.lock = threading.Lock()
        self.sticky = {}
        self.seen_from_quest = set()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", self.port))
        self.server.listen(4)
        self.server.settimeout(0.5)
        threading.Thread(target=self._accept, daemon=True).start()
        self.log.info(f"waiting for the Quest on tcp port {self.port}")

    def _accept(self):
        while not self._stop.is_set():
            try:
                sock, addr = self.server.accept()
            except (socket.timeout, OSError):
                continue
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            client = Client(sock, addr, self.log)
            with self.lock:
                self.clients.append(client)
            for line in list(self.sticky.values()):
                client.send(line)
            threading.Thread(target=self._read, args=(client,), daemon=True).start()
            self.log.info(f"quest {addr[0]} connected")

    def _read(self, client):
        buf = b""
        while client.alive:
            try:
                chunk = client.sock.recv(65536)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.strip():
                    self._handle(line)
        client.close()

    def _handle(self, line):
        try:
            packet = json.loads(line)
            topic = packet["topic"]
            if topic not in FROM_QUEST:
                self.log.warning(f"ignored topic {topic!r} from the Quest")
                return
            msg = decode(packet["msg"])
            if topic not in self.seen_from_quest:
                self.seen_from_quest.add(topic)
                self.log.info(f"receiving {topic} from the Quest")
            if topic == Topic.HAND and getattr(msg, "tracked", False) and ("hand", msg.side) not in self.seen_from_quest:
                self.seen_from_quest.add(("hand", msg.side))
                self.log.info(f"{msg.side} hand tracked")
            self.publish(topic, msg)
        except Exception as e:
            self.log.warning(f"bad packet from the Quest: {e}")

    def step(self):
        latest = {}
        ordered = []
        for topic, msg in self.sub.drain():
            if topic in LATEST_ONLY:
                latest[topic] = msg
            else:
                ordered.append((topic, msg))
        ordered += list(latest.items())
        if not ordered:
            return
        lines = []
        for topic, msg in ordered:
            line = (json.dumps({"topic": topic, "msg": encode(msg)}, separators=(",", ":")) + "\n").encode()
            if topic in STICKY:
                self.sticky[topic] = line
            lines.append(line)
        with self.lock:
            self.clients = [c for c in self.clients if c.alive]
            clients = list(self.clients)
        for c in clients:
            for line in lines:
                c.send(line)

    def teardown(self):
        with self.lock:
            for c in self.clients:
                c.close()
        self.server.close()
