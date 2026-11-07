#!/usr/bin/env python3
"""Compose the README three-panel figure: image | fused cloud | occupancy grid."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


PANEL_W = 600
PANEL_H = 400


def read_image(path: Path) -> np.ndarray:
    # cv2.imread is unreliable for non-ASCII Windows paths; decode bytes instead.
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"failed to read {path}")
    return image


def write_image(path: Path, image: np.ndarray) -> None:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise SystemExit(f"failed to encode {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded.tofile(str(path))


def letterbox(image: np.ndarray) -> np.ndarray:
    scale = min(PANEL_W / image.shape[1], PANEL_H / image.shape[0])
    width = max(1, int(round(image.shape[1] * scale)))
    height = max(1, int(round(image.shape[0] * scale)))
    resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)
    x = (PANEL_W - width) // 2
    y = (PANEL_H - height) // 2
    canvas[y:y + height, x:x + width] = resized
    return canvas


def label(image: np.ndarray, text: str) -> None:
    cv2.rectangle(image, (0, 0), (PANEL_W, 36), (0, 0, 0), -1)
    cv2.putText(image, text, (14, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.72,
                (255, 255, 255), 2, cv2.LINE_AA)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--fused", type=Path, required=True)
    parser.add_argument("--grid", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    panels = [
        (letterbox(read_image(args.image)), "KITTI frame"),
        (letterbox(read_image(args.fused)), "Fused RGB cloud"),
        (letterbox(read_image(args.grid)), "2D occupancy grid"),
    ]
    for panel, title in panels:
        label(panel, title)

    output = np.hstack([panel for panel, _ in panels])
    write_image(args.out, output)
    print(f"wrote {args.out} ({output.shape[1]}x{output.shape[0]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
