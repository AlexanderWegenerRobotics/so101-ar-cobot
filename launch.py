import argparse
import importlib
import logging
import time
from pathlib import Path

import yaml

from interfaces.bus import Broker, load_bus_config

ROOT = Path(__file__).resolve().parent
DEFAULT_LAUNCH = ROOT / "config" / "launch.yaml"


def load_node(spec, bus_cfg):
    module_name, cls_name = spec["node"].split(":")
    cls = getattr(importlib.import_module(module_name), cls_name)
    return cls(bus_cfg=bus_cfg, **spec.get("params", {}))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", nargs="?", default="dev")
    parser.add_argument("--config", default=str(DEFAULT_LAUNCH))
    parser.add_argument("--no-broker", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s", datefmt="%H:%M:%S")

    launch = yaml.safe_load(Path(args.config).read_text())
    profile = launch["profiles"][args.profile]
    bus_cfg = load_bus_config()
    if not args.no_broker and profile.get("broker", True):
        Broker(bus_cfg).start()
        time.sleep(0.2)

    if "sim" in profile:
        from hardware import get_sim_world
        world = get_sim_world(**profile["sim"])
        world.start()
        logging.info("sim world running")

    nodes = [load_node(spec, bus_cfg) for spec in profile["nodes"]]
    for n in nodes:
        n.start()
        logging.info(f"started {n.name}")
    try:
        while any(n.alive for n in nodes):
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for n in reversed(nodes):
            n.stop()


if __name__ == "__main__":
    main()
