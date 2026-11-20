from interfaces.messages import Topic
from interfaces.node import Node


class SimControl(Node):
    name = "sim_control"
    rate_hz = 20.0
    subscribe = [Topic.OPERATOR_COMMAND]

    def setup(self):
        from hardware import get_sim_world
        self.world = get_sim_world()

    def step(self):
        for _, cmd in self.sub.drain():
            if cmd.kind == "reset_world":
                self.world.reset()
                self.log.info("cubes and arm reset to the start (the task node moved the arm home first)")
