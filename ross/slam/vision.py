"""Per-frame vision: MiDaS depth, YOLO human detection, Re-ID tracking."""

import numpy as np
from scipy.spatial.distance import cosine

YOLO_DEFAULT_MODEL = "yolov8n.pt"
YOLO_CONF = 0.40
PERSON_CLASS_ID = 0

REID_APPEARANCE_THRESH = 0.82
SAME_PERSON_DIST = 1.2


def load_midas(model_type: str = "DPT_Hybrid"):
    """Load a MiDaS model + input transform, on the best available device."""
    import torch

    print(f"[MiDaS] Loading {model_type} ...")
    model = torch.hub.load("intel-isl/MiDaS", model_type, trust_repo=True)
    transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
    transform = (
        transforms.dpt_transform if "DPT" in model_type else transforms.small_transform
    )
    device = (
        torch.device("mps")
        if torch.backends.mps.is_available()
        else torch.device("cuda")
        if torch.cuda.is_available()
        else torch.device("cpu")
    )
    print(f"[MiDaS] Device: {device}")
    model.to(device).eval()
    return model, transform, device


def estimate_depth(frame_bgr, model, transform, device):
    """Run MiDaS on a BGR frame; return a normalised 2-D depth map."""
    import cv2
    import torch

    img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    batch = transform(img).to(device)
    with torch.no_grad():
        pred = model(batch)
        pred = torch.nn.functional.interpolate(
            pred.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()
    d = pred.cpu().numpy().astype(np.float32)
    lo, hi = d.min(), d.max()
    if hi - lo > 1e-6:
        d = (d - lo) / (hi - lo)
    return d


def load_yolo(weights: str | None = None):
    """Load a YOLO model. Returns None if ultralytics isn't installed."""
    try:
        from ultralytics import YOLO

        target = weights or YOLO_DEFAULT_MODEL
        print(f"[YOLO] Loading {target} ...")
        model = YOLO(target)
        print("[YOLO] Ready.")
        return model
    except ImportError:
        print("[YOLO] ultralytics not installed — continuing without human detection")
        return None


def detect_people(frame_bgr, yolo_model) -> list[dict]:
    """Run YOLO and return a list of {x1,y1,x2,y2,conf,cx,cy} dicts."""
    if yolo_model is None:
        return []
    results = yolo_model(
        frame_bgr, conf=YOLO_CONF, classes=[PERSON_CLASS_ID], verbose=False
    )
    people = []
    for r in results:
        for box in r.boxes:
            if int(box.cls[0]) != PERSON_CLASS_ID:
                continue
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            people.append(
                {
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "conf": float(box.conf[0]),
                    "cx": (x1 + x2) // 2,
                    "cy": (y1 + y2) // 2,
                }
            )
    return people


def appearance_embedding(frame_bgr, person: dict) -> np.ndarray:
    """L2-normalised 48-D BGR histogram for Re-ID (torso crop)."""
    x1, y1, x2, y2 = person["x1"], person["y1"], person["x2"], person["y2"]
    h_box, w_box = y2 - y1, x2 - x1

    y_top = y1 + h_box // 3
    y_bot = y2 - h_box // 3
    x_lft = x1 + w_box // 6
    x_rgt = x2 - w_box // 6

    crop = frame_bgr[max(0, y_top) : max(0, y_bot), max(0, x_lft) : max(0, x_rgt)]
    if crop.size == 0:
        crop = frame_bgr[max(0, y1) : max(0, y2), max(0, x1) : max(0, x2)]
    if crop.size == 0:
        return np.zeros(48, dtype=np.float32)

    hist = []
    for ch in range(3):
        h, _ = np.histogram(crop[:, :, ch], bins=16, range=(0, 256))
        hist.append(h.astype(np.float32))
    vec = np.concatenate(hist)
    norm = np.linalg.norm(vec)
    return vec / (norm + 1e-6)


class PersonTracker:
    """Online Re-ID: merge detections by appearance similarity + 3-D distance."""

    def __init__(self):
        self.people: list[dict] = []

    def update(self, pos3d: tuple[float, float, float], embedding: np.ndarray) -> dict:
        best_idx, best_sim = -1, -1.0
        for idx, p in enumerate(self.people):
            sim = 1.0 - cosine(embedding, p["embedding"])
            if sim < REID_APPEARANCE_THRESH:
                continue
            dist = np.linalg.norm(np.array(pos3d) - np.array(p["pos3d"]))
            if dist > SAME_PERSON_DIST:
                continue
            if sim > best_sim:
                best_sim, best_idx = sim, idx

        if best_idx >= 0:
            p = self.people[best_idx]
            n = p["count"]
            p["pos3d"] = tuple(
                (np.array(p["pos3d"]) * n + np.array(pos3d)) / (n + 1)
            )
            p["embedding"] = (p["embedding"] * n + embedding) / (n + 1)
            p["embedding"] /= np.linalg.norm(p["embedding"]) + 1e-6
            p["count"] += 1
            return p

        label = f"Person {len(self.people) + 1}"
        new_p = {
            "pos3d": pos3d,
            "embedding": embedding.copy(),
            "count": 1,
            "label": label,
        }
        self.people.append(new_p)
        print(
            f"    [Re-ID] New: {label}  "
            f"pos=({pos3d[0]:.2f}, {pos3d[1]:.2f}, {pos3d[2]:.2f})"
        )
        return new_p

    def all_people(self) -> list[dict]:
        return self.people
