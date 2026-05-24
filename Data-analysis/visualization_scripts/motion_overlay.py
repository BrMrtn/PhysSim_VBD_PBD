"""Composite every Nth frame of an image sequence into a single motion-trail image.

The background is static and a single object moves across it. For each pixel we
keep the value that deviates most from the per-pixel median (the background),
which lays every sampled position of the object onto one image regardless of
whether the object is lighter or darker than the background.
"""

import glob
import os

import cv2
import numpy as np

src_dir = r"D:\uni\Szakdoga_teljes\PhysSim_VBD_PBD\Data-analysis\Logs\StiffnessRatio\Newton128ss"
dst_path = r"D:\uni\Szakdoga_teljes\PhysSim_VBD_PBD\Data-analysis\Visuals\StiffnessRatio\Newton128StiffnessRatio.png"
step = 10

def build_overlay(src_dir: str, step: int) -> np.ndarray:
    paths = sorted(glob.glob(os.path.join(src_dir, "*.png")))
    if not paths:
        raise FileNotFoundError(f"No PNG images found in {src_dir}")

    sampled = paths[::step]
    print(f"Found {len(paths)} images, using every {step}th -> {len(sampled)} frames")

    frames = []
    for p in sampled:
        img = cv2.imread(p, cv2.IMREAD_COLOR)
        if img is None:
            print(f"  skipping unreadable file: {p}")
            continue
        frames.append(img)

    if not frames:
        raise RuntimeError("No images could be read")

    stack = np.stack(frames).astype(np.float32)  # (T, H, W, 3)

    # Per-pixel background estimate = median over time.
    background = np.median(stack, axis=0)  # (H, W, 3)

    # For each frame/pixel, how far it deviates from the background (sum over channels).
    deviation = np.abs(stack - background[None]).sum(axis=3)  # (T, H, W)

    # Index of the frame with the largest deviation at each pixel.
    best = np.argmax(deviation, axis=0)  # (H, W)

    h, w = best.shape
    rows, cols = np.indices((h, w))
    result = stack[best, rows, cols]  # (H, W, 3)

    return result.astype(np.uint8)


overlay = build_overlay(src_dir, step)

os.makedirs(os.path.dirname(dst_path), exist_ok=True)
if not cv2.imwrite(dst_path, overlay):
    raise RuntimeError(f"Failed to write {dst_path}")
print(f"Saved {dst_path}")
