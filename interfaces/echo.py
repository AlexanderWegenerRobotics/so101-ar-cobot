import argparse
import json

from interfaces.bus import Subscriber
from interfaces.messages import GROUND_TRUTH_PREFIX, Topic, to_jsonable


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("topics", nargs="*")
    parser.add_argument("--full", action="store_true")
    args = parser.parse_args()
    topics = args.topics or [v for k, v in vars(Topic).items()
                             if not k.startswith("_") and not v.startswith(GROUND_TRUTH_PREFIX)]
    sub = Subscriber(topics, allow_ground_truth=True)
    print(f"listening on {topics}")
    try:
        while True:
            r = sub.recv(timeout_ms=1000)
            if r is None:
                continue
            topic, msg = r
            text = json.dumps(to_jsonable(msg))
            print(f"[{topic}] {type(msg).__name__} {text if args.full else text[:160]}")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
