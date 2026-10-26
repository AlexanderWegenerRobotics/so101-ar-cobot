import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum

import cv2
import mujoco
import numpy as np

from hardware.sim_bus import DeviceAlreadyConnectedError, DeviceNotConnectedError
from sim.scene_builder import camera_intrinsics

logger = logging.getLogger(__name__)

ROTATIONS = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, -90: cv2.ROTATE_90_COUNTERCLOCKWISE}


class ColorMode(str, Enum):
    RGB = "rgb"
    BGR = "bgr"


@dataclass
class SimRealSenseCameraConfig:
    serial_number_or_name: str = "sim"
    fps: int | None = 30
    width: int | None = 1920
    height: int | None = 1080
    color_mode: ColorMode = ColorMode.RGB
    use_rgb: bool = True
    use_depth: bool = False
    rotation: int = 0
    warmup_s: int = 1
    exposure: int | None = None
    gain: int | None = None
    white_balance: int | None = None

    def __post_init__(self):
        self.color_mode = ColorMode(self.color_mode)
        if not self.use_rgb and not self.use_depth:
            raise ValueError("At least one of `use_rgb` or `use_depth` must be enabled.")


class SimRealSenseCamera:
    def __init__(self, config, world):
        self.config = config
        self.world = world
        cam = world.manifest["camera"]
        self.color_cam = cam["name"]
        self.depth_cfg = cam.get("depth")
        self.serial_number = config.serial_number_or_name
        self.fps = config.fps or cam["fps"]
        self.width = config.width or cam["resolution"][0]
        self.height = config.height or cam["resolution"][1]
        self.color_mode = config.color_mode
        self.use_rgb = config.use_rgb
        self.use_depth = config.use_depth
        self.warmup_s = config.warmup_s
        self.rotation = ROTATIONS.get(int(config.rotation))
        if self.use_depth and self.depth_cfg is None:
            raise ValueError("use_depth=True but the scene has no depth camera, add camera.depth to scene.yaml")
        if self.use_depth:
            max_w, max_h = self.depth_cfg["resolution"]
            if self.width > max_w or self.height > max_h:
                self._config_error = (f"depth stream does not support {self.width}x{self.height}, "
                                      f"the D435i depth sensor goes up to {max_w}x{max_h}")
        self.thread = None
        self.stop_event = None
        self.frame_lock = threading.Lock()
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.latest_timestamp = None
        self.new_frame_event = threading.Event()
        self._ready = threading.Event()
        self._error = None
        if not hasattr(self, "_config_error"):
            self._config_error = None

    def __str__(self):
        return f"{self.__class__.__name__}({self.serial_number})"

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()

    @property
    def is_connected(self):
        return self.thread is not None and self.thread.is_alive()

    @staticmethod
    def find_cameras():
        return [{
            "name": "Sim Intel RealSense D435I", "type": "RealSense", "id": "sim",
            "firmware_version": "sim", "usb_type_descriptor": "sim", "physical_port": "sim",
            "product_id": "sim", "product_line": "D400",
        }]

    def connect(self, warmup=True):
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")
        if self._config_error:
            raise ConnectionError(f"Failed to open {self}: {self._config_error}")
        if not self.world.is_running:
            self.world.start()
        self.stop_event = threading.Event()
        self._ready.clear()
        self.thread = threading.Thread(target=self._read_loop, name=f"{self}_read_loop", daemon=True)
        self.thread.start()
        self._ready.wait(timeout=10)
        if self._error is not None:
            raise ConnectionError(f"Failed to open {self}: {self._error}")
        if warmup:
            deadline = time.perf_counter() + self.warmup_s
            while time.perf_counter() < deadline:
                self.async_read(timeout_ms=1000) if self.use_rgb else self.async_read_depth(timeout_ms=1000)
                time.sleep(0.05)
        logger.info(f"{self} connected.")

    def _render(self, renderer, camera):
        with self.world.lock:
            renderer.update_scene(self.world.data, camera=camera)
        return renderer.render()

    def _postprocess(self, image, depth_frame=False):
        if not depth_frame and self.color_mode == ColorMode.BGR:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if self.rotation is not None:
            image = cv2.rotate(image, self.rotation)
        if depth_frame and image.ndim == 2:
            image = image[..., np.newaxis]
        return image

    def _depth_to_mm(self, depth_m):
        lo, hi = self.depth_cfg["range"]
        mm = np.round(depth_m * 1000).astype(np.uint16)
        mm[(depth_m < lo) | (depth_m > hi)] = 0
        return mm

    def _read_loop(self):
        try:
            color_r = mujoco.Renderer(self.world.model, self.height, self.width) if self.use_rgb else None
            depth_r = None
            if self.use_depth:
                depth_r = mujoco.Renderer(self.world.model, self.height, self.width)
                depth_r.enable_depth_rendering()
        except Exception as e:
            self._error = e
            self._ready.set()
            return
        self._ready.set()
        period = 1.0 / self.fps
        next_t = time.perf_counter()
        while not self.stop_event.is_set():
            color = self._postprocess(self._render(color_r, self.color_cam)) if color_r else None
            depth = None
            if depth_r:
                depth = self._postprocess(self._depth_to_mm(self._render(depth_r, self.depth_cfg["name"])), True)
            with self.frame_lock:
                if self.stop_event.is_set():
                    break
                if color is not None:
                    self.latest_color_frame = color
                if depth is not None:
                    self.latest_depth_frame = depth
                self.latest_timestamp = time.perf_counter()
            self.new_frame_event.set()
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()
        for r in (color_r, depth_r):
            if r is not None:
                r.close()

    def _check_connected(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

    def _async_read(self, timeout_ms, read_depth=False):
        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            raise TimeoutError(f"Timed out waiting for frame from camera {self} after {timeout_ms} ms.")
        with self.frame_lock:
            frame = self.latest_depth_frame if read_depth else self.latest_color_frame
            self.new_frame_event.clear()
        if frame is None:
            raise RuntimeError(f"Internal error: Event set but no frame available for {self}.")
        return frame

    def _read_latest(self, max_age_ms, read_depth=False):
        with self.frame_lock:
            frame = self.latest_depth_frame if read_depth else self.latest_color_frame
            timestamp = self.latest_timestamp
        if frame is None or timestamp is None:
            raise RuntimeError(f"{self} has not captured any frames yet.")
        age_ms = (time.perf_counter() - timestamp) * 1e3
        if age_ms > max_age_ms:
            raise TimeoutError(f"{self} latest frame is too old: {age_ms:.1f} ms (max allowed: {max_age_ms} ms).")
        return frame

    def _need(self, rgb):
        if rgb and not self.use_rgb:
            raise RuntimeError(f"{self}: cannot read color — camera was configured with use_rgb=False.")
        if not rgb and not self.use_depth:
            raise RuntimeError(f"Failed to capture depth frame. Depth stream is not enabled for {self}.")

    def read(self, color_mode=None, timeout_ms=0):
        self._check_connected()
        self._need(True)
        self.new_frame_event.clear()
        return self._async_read(10000)

    def read_depth(self, timeout_ms=200):
        self._check_connected()
        self._need(False)
        self.new_frame_event.clear()
        return self._async_read(10000, read_depth=True)

    def async_read(self, timeout_ms=200):
        self._check_connected()
        self._need(True)
        return self._async_read(timeout_ms)

    def async_read_depth(self, timeout_ms=200):
        self._check_connected()
        self._need(False)
        return self._async_read(timeout_ms, read_depth=True)

    def read_latest(self, max_age_ms=500):
        self._check_connected()
        self._need(True)
        return self._read_latest(max_age_ms)

    def read_latest_depth(self, max_age_ms=500):
        self._check_connected()
        self._need(False)
        return self._read_latest(max_age_ms, read_depth=True)

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} not connected.")
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.thread = None
        with self.frame_lock:
            self.latest_color_frame = None
            self.latest_depth_frame = None
            self.latest_timestamp = None
        self.new_frame_event.clear()
        logger.info(f"{self} disconnected.")

    def intrinsics(self, depth=False):
        if depth:
            w, h, fovy = self.width, self.height, self.depth_cfg["fovy_deg"]
        else:
            w, h, fovy = self.width, self.height, self.world.manifest["camera"]["fovy_deg"]
        if self.rotation in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
            raise NotImplementedError("intrinsics for rotated images are not provided")
        return {"width": w, "height": h, **camera_intrinsics(fovy, w, h), "coeffs": [0.0] * 5}
