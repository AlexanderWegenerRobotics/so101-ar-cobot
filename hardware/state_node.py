from interfaces.messages import ArmState, Topic
from interfaces.node import Node


class ArmStateStub(Node):
    name = "arm_state_stub"
    rate_hz = 30.0

    def setup(self):
        from hardware import make_robot
        self.robot = make_robot(self.params.get("backend"))
        self.robot.connect()

    def step(self):
        obs = self.robot.get_observation()
        joints = {k.removesuffix(".pos"): v for k, v in obs.items() if k.endswith(".pos")}
        self.publish(Topic.ARM_STATE, ArmState(joints=joints, status="idle"))

    def teardown(self):
        if self.robot.is_connected:
            self.robot.disconnect()
