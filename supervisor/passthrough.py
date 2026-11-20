"""Placeholder safety supervisor: forwards every arm/goal to arm/command and publishes a clear safety state.

The controller stops the arm when it hears no safety message for 0.2 s, so something has to publish one. Replace
this with the real supervisor (hand distance zones -> speed_scale / stop) without touching controls.
"""
from interfaces.messages import ArmGoal, SafetyState, Topic
from interfaces.node import Node


class PassThroughSupervisor(Node):
    name = "supervisor"
    rate_hz = 30.0
    subscribe = [Topic.ARM_GOAL]

    def setup(self):
        self.speed_scale = float(self.params.get("speed_scale", 1.0))
        self.log.warning("pass-through supervisor: no safety checks, goals go straight to the arm")

    def step(self):
        for topic, msg in self.sub.drain():
            if isinstance(msg, ArmGoal):
                self.publish(Topic.ARM_COMMAND, msg)
        self.publish(Topic.SAFETY, SafetyState(speed_scale=self.speed_scale, reason="pass-through supervisor"))
