from dataclasses import dataclass

from interfaces.transforms import Pose

FRAME_BASE = "base"


@dataclass
class ObjectState:
    tag_id: int
    name: str
    kind: str
    pose: Pose
    stamp: float
    source: str
    frame: str = FRAME_BASE

    def to_dict(self):
        return {"tag_id": self.tag_id, "name": self.name, "kind": self.kind, "frame": self.frame,
                "stamp": self.stamp, "source": self.source, **self.pose.to_dict()}

    @classmethod
    def from_dict(cls, d):
        return cls(d["tag_id"], d["name"], d["kind"], Pose(d["pos"], d["quat"]),
                   d["stamp"], d["source"], d.get("frame", FRAME_BASE))
