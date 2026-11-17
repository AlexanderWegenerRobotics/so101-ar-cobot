import time

from controls.arm_controller import ArmController
from controls.kinematics import load_kinematics
from interfaces.messages import Topic
from interfaces.node import Node


class ArmControllerNode(Node):
    name = "arm_controller"
    subscribe = [Topic.ARM_COMMAND, Topic.SAFETY]

    def setup(self):
        from hardware import load_robot_config, make_robot
        cfg = load_robot_config()
        self.rate_hz = cfg["controller"]["rate_hz"]
        self.robot = make_robot(cfg=cfg)
        self.robot.connect()
        self.controller = ArmController(load_kinematics(), cfg["sim"]["joint_map"],
                                        cfg["home_qpos"][:5], cfg["controller"])

    def step(self):
        now = time.time()
        observation = self.robot.get_observation()
        joints = {key.removesuffix(".pos"): value for key, value in observation.items()}
        messages = []
        for topic, msg in self.sub.drain():
            if topic == Topic.SAFETY:
                self.controller.on_safety(msg, now)
            else:
                messages += self.controller.on_goal(msg, joints, now)
        action, update_messages = self.controller.update(joints, now)
        self.robot.send_action({f"{name}.pos": value for name, value in action.items()})
        for topic, msg in messages + update_messages:
            self.publish(topic, msg)

    def teardown(self):
        if self.robot.is_connected:
            self.robot.disconnect()
