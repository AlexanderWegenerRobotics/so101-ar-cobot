from interfaces.messages import ObjectArray, Topic
from interfaces.node import Node
from perception import make_object_tracker


class ObjectPublisher(Node):
    name = "object_publisher"
    rate_hz = 30.0

    def setup(self):
        self.rate_hz = self.params.get("rate_hz", self.rate_hz)
        self.tracker = make_object_tracker(self.params.get("source"))

    def step(self):
        objects = self.tracker.get_objects()
        self.publish(Topic.OBJECTS, ObjectArray(objects=list(objects.values()), source=self.tracker.source))
