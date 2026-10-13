"""
Downward-camera feed and person / hard-hat detectors for the NIDAR Gazebo sim.

CameraFeed subscribes to one drone's Gazebo topics (no ROS needed):
    /nidar/drone<i>/camera   gz.msgs.Image                         RGB frames
    /nidar/drone<i>/boxes    gz.msgs.AnnotatedAxisAligned2DBox_V    ground-truth boxes
and pairs each frame with the boxes rendered in the same sim step.

Detectors turn a frame into a list of Detection(cls, conf, u, v, box):

    gt     reads the bounding-box camera. Perfect labels (1 = person, 2 = hardhat),
           so the mission logic can be tested without a trained model.
    yolo   runs an Ultralytics model on the RGB frame. Classes whose name contains
           "person"/"people"/"pedestrian"/"human" count as person; "hat"/"helmet"
           count as hardhat. COCO yolov8n only knows "person"; train your own
           weights for hard hats (see --dump-dataset in swarm_mission.py).

DatasetDumper writes RGB frames plus YOLO-format labels from the ground-truth
boxes, so the sim can generate training data for the real detector.
"""

import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

CLASS_NAMES = {1: "person", 2: "hardhat"}
YOLO_CLASS_IDS = {"person": 0, "hardhat": 1}      # for dumped datasets


@dataclass
class Detection:
    cls: str
    conf: float
    u: float              # box centre, pixels
    v: float
    box: tuple            # x0, y0, x1, y1


@dataclass
class Frame:
    image: np.ndarray     # BGR
    stamp: float          # sim time, s
    boxes: list           # [(label, x0, y0, x1, y1)] from the bbox camera, or None
    received: float       # wall clock


def _stamp(header):
    return header.stamp.sec + header.stamp.nsec * 1e-9


class CameraFeed:
    """Latest paired (image, gt boxes) frame for one drone."""

    def __init__(self, node, drone_index):
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V

        self._lock = threading.Lock()
        self._boxes = {}            # stamp -> box list, last few only
        self._frame = None
        self._seq = 0
        base = f"/nidar/drone{drone_index}"
        if not node.subscribe(Image, f"{base}/camera", self._on_image):
            raise RuntimeError(f"could not subscribe to {base}/camera")
        if not node.subscribe(AnnotatedAxisAligned2DBox_V, f"{base}/boxes", self._on_boxes):
            raise RuntimeError(f"could not subscribe to {base}/boxes")

    def _on_boxes(self, msg):
        boxes = [(b.label, b.box.min_corner.x, b.box.min_corner.y, b.box.max_corner.x, b.box.max_corner.y)
                 for b in msg.annotated_box]
        with self._lock:
            self._boxes[round(_stamp(msg.header), 4)] = boxes
            if len(self._boxes) > 20:
                for k in sorted(self._boxes)[:-20]:
                    del self._boxes[k]

    def _on_image(self, msg):
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        stamp = round(_stamp(msg.header), 4)
        with self._lock:
            self._frame = Frame(bgr, stamp, None, time.time())
            self._seq += 1

    def latest(self):
        """(seq, Frame) with gt boxes attached when the matching message has arrived."""
        with self._lock:
            frame = self._frame
            if frame is not None and frame.boxes is None:
                frame.boxes = self._boxes.get(frame.stamp)
            return self._seq, frame


class GroundTruthDetector:
    name = "gt"

    def __init__(self, min_box_px=6):
        self.min_box_px = min_box_px

    def detect(self, frame):
        out = []
        for label, x0, y0, x1, y1 in frame.boxes or ():
            cls = CLASS_NAMES.get(label)
            if cls is None or min(x1 - x0, y1 - y0) < self.min_box_px:
                continue
            out.append(Detection(cls, 1.0, (x0 + x1) / 2, (y0 + y1) / 2, (x0, y0, x1, y1)))
        return out

    def ready(self, frame):
        return frame.boxes is not None


class YoloDetector:
    name = "yolo"

    def __init__(self, weights, conf=0.35, imgsz=640):
        from ultralytics import YOLO
        self.model = YOLO(str(weights))
        self.conf = conf
        self.imgsz = imgsz
        self.class_map = {}
        for idx, name in self.model.names.items():
            n = name.lower()
            if any(k in n for k in ("person", "people", "pedestrian", "human")):
                self.class_map[idx] = "person"
            elif any(k in n for k in ("hat", "helmet")):
                self.class_map[idx] = "hardhat"
        if not self.class_map:
            raise ValueError(f"{weights}: no person / hardhat classes in {self.model.names}")
        print(f"[detector] YOLO {weights}: using classes "
              f"{ {self.model.names[i]: c for i, c in self.class_map.items()} }")

    def detect(self, frame):
        result = self.model.predict(frame.image, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        out = []
        for xyxy, c, k in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), result.boxes.cls.tolist()):
            cls = self.class_map.get(int(k))
            if cls:
                x0, y0, x1, y1 = xyxy
                out.append(Detection(cls, float(c), (x0 + x1) / 2, (y0 + y1) / 2, (x0, y0, x1, y1)))
        return out

    def ready(self, frame):
        return True


def make_detector(kind, weights=None, conf=0.35):
    if kind == "gt":
        return GroundTruthDetector()
    if kind == "yolo":
        return YoloDetector(weights or Path.home() / "models" / "yolov8n.pt", conf=conf)
    raise ValueError(f"unknown detector {kind!r}")


class DatasetDumper:
    """Saves every n-th paired frame as images/<name>.jpg + labels/<name>.txt (YOLO format)."""

    def __init__(self, root, every_n=5, keep_empty=0.1):
        self.root = Path(root)
        (self.root / "images").mkdir(parents=True, exist_ok=True)
        (self.root / "labels").mkdir(parents=True, exist_ok=True)
        (self.root / "data.yaml").write_text(
            f"path: {self.root.resolve()}\ntrain: images\nval: images\n"
            f"names:\n  0: person\n  1: hardhat\n")
        self.every_n = every_n
        self.keep_empty = keep_empty
        self.count = {}
        self.saved = 0
        self._rng = np.random.default_rng(0)

    def maybe_save(self, drone, frame):
        if frame.boxes is None:
            return
        n = self.count[drone] = self.count.get(drone, 0) + 1
        if n % self.every_n:
            return
        h, w = frame.image.shape[:2]
        lines = []
        for label, x0, y0, x1, y1 in frame.boxes:
            cls = CLASS_NAMES.get(label)
            if cls is None or min(x1 - x0, y1 - y0) < 4:
                continue
            lines.append(f"{YOLO_CLASS_IDS[cls]} {(x0 + x1) / 2 / w:.6f} {(y0 + y1) / 2 / h:.6f} "
                         f"{(x1 - x0) / w:.6f} {(y1 - y0) / h:.6f}")
        if not lines and self._rng.random() > self.keep_empty:
            return
        name = f"d{drone}_{frame.stamp:010.3f}".replace(".", "_")
        cv2.imwrite(str(self.root / "images" / f"{name}.jpg"), frame.image)
        (self.root / "labels" / f"{name}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
        self.saved += 1
