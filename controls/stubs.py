from interfaces.messages import SafetyState, Topic
from interfaces.node import Node


class SafetyStub(Node):
    name = "safety_stub"
    rate_hz = 30.0

    def step(self):
        self.publish(Topic.SAFETY, SafetyState(speed_scale=self.params.get("speed_scale", 1.0)))
