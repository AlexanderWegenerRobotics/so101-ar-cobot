import json
import time

from interfaces.bus import Subscriber
from interfaces.messages import Topic
from task.objects import DEFAULT_MANIFEST, load_objects, target_bin

WAIT_PERIOD = 0.1
MAX_AGE = 2.0


def inside(position, bin_position, half_size):
    return all(abs(position[axis] - bin_position[axis]) < half_size for axis in (0, 1))


def main():
    cubes, bins = load_objects()
    half_size = json.loads(DEFAULT_MANIFEST.read_text())["bins"][0]["inner_size"][0] / 2
    subscriber = Subscriber([Topic.OBJECTS])
    message = None
    while message is None:
        time.sleep(WAIT_PERIOD)
        message = subscriber.latest(Topic.OBJECTS, max_age=MAX_AGE)
    objects = {obj.tag_id: obj for obj in message.objects}
    for tag, cube in cubes.items():
        position = objects[tag].pose.pos
        bin_position = objects[target_bin(cube, bins).tag_id].pose.pos
        if inside(position, bin_position, half_size):
            state = "in its bin"
        elif not cube.robot_feasible:
            state = "left for the human"
        else:
            state = "NOT in its bin"
        print(f"cube {tag} {cube.color:5s} x {position[0]:6.3f} y {position[1]:6.3f}  {state}")


if __name__ == "__main__":
    main()
