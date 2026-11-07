#!/usr/bin/env python3
"""Generate the tiny synthetic KITTI-style fixture used by M0 data checks.

This is NOT real KITTI data. It exists so the repository has a deterministic
single-frame sample before the real dataset is downloaded. The calibration uses
a simple Velodyne -> camera transform:
    camera_x = -velodyne_y
    camera_y = -velodyne_z
    camera_z =  velodyne_x

Replace `data/sample` with a real KITTI object frame for projection-quality
tests. Keep the same directory layout:
    data/sample/image_2/000000.png
    data/sample/velodyne/000000.bin
    data/sample/calib/000000.txt
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


WIDTH = 1242
HEIGHT = 375


def make_image() -> np.ndarray:
    image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    for y in range(HEIGHT):
        t = y / max(HEIGHT - 1, 1)
        image[y, :] = (
            int(190 - 70 * t),
            int(150 - 45 * t),
            int(110 - 25 * t),
        )

    # Road.
    road = np.array(
        [[0, HEIGHT - 1], [WIDTH - 1, HEIGHT - 1], [900, 210], [330, 210]],
        dtype=np.int32,
    )
    cv2.fillPoly(image, [road], (70, 78, 82))

    # A few high-contrast objects to make the fixture useful for overlays.
    cv2.rectangle(image, (420, 150), (520, 260), (40, 40, 200), -1)
    cv2.rectangle(image, (700, 120), (820, 250), (40, 160, 60), -1)
    cv2.rectangle(image, (880, 170), (930, 260), (180, 60, 40), -1)
    cv2.putText(
        image,
        "SYNTHETIC SAMPLE - NOT KITTI",
        (30, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return image


def pixel_to_velodyne(u: np.ndarray, v: np.ndarray, depth: np.ndarray) -> tuple[np.ndarray, ...]:
    """Inverse of the synthetic P2 / Tr_velo_to_cam chain.

    P2 projects camera (x right, y down, z forward) with f=500, cx=621, cy=187.5.
    Tr_velo_to_cam maps camera_x=-velo_y, camera_y=-velo_z, camera_z=velo_x.
    """
    x_cam = (u - 621.0) * depth / 500.0
    y_cam = (v - 187.5) * depth / 500.0
    return depth, -x_cam, -y_cam


def surface_points(
    rect: tuple[int, int, int, int],
    depth: float,
    count: int,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Points on a camera-facing rectangle, at a fixed approximate depth."""
    x0, y0, x1, y1 = rect
    u = rng.uniform(x0, x1, count)
    v = rng.uniform(y0, y1, count)
    d = depth + rng.normal(0.0, 0.18, count)

    x, y, z = pixel_to_velodyne(u, v, d)
    # A little thickness/noise so the cloud is not a perfect plane.
    x += rng.normal(0.0, 0.03, count)
    y += rng.normal(0.0, 0.03, count)
    z += rng.normal(0.0, 0.03, count)
    return x, y, z


def make_cloud() -> np.ndarray:
    rng = np.random.default_rng(42)

    # Three camera-facing object surfaces, aligned with the rectangles in the
    # synthetic image. Counts sum to 20000 points so the fixture is dense
    # enough for a healthy LiDAR confidence score.
    xr, yr, zr = surface_points((420, 150, 520, 260), 15.0, 3000, rng)
    xg, yg, zg = surface_points((700, 120, 820, 250), 21.0, 3500, rng)
    xb, yb, zb = surface_points((880, 170, 930, 260), 27.0, 2000, rng)

    # Ground plane, deliberately below the M4 height filter.
    n_ground = 8500
    x_ground = rng.uniform(5.0, 45.0, n_ground)
    y_ground = rng.uniform(-12.0, 12.0, n_ground)
    z_ground = np.full(n_ground, -1.65)

    # A sparse ring behind the camera, so the sample exercises a 360-degree
    # LiDAR azimuth coverage instead of only the forward camera sector.
    n_ring = 3000
    ring_angle = np.pi * 0.5 + np.pi * np.arange(n_ring) / n_ring
    ring_range = rng.uniform(10.0, 30.0, n_ring)
    x_ring = ring_range * np.cos(ring_angle)
    y_ring = ring_range * np.sin(ring_angle)
    z_ring = np.full(n_ring, 0.5)

    x = np.concatenate([xr, xg, xb, x_ground, x_ring])
    y = np.concatenate([yr, yg, yb, y_ground, y_ring])
    z = np.concatenate([zr, zg, zb, z_ground, z_ring])
    intensity = rng.uniform(0.0, 1.0, x.size)

    return np.column_stack([x, y, z, intensity]).astype(np.float32)


def make_calibration() -> str:
    # P2 is a 3x4 projection matrix for the left color camera.
    # The transform maps Velodyne (x forward, y left, z up) to
    # camera (x right, y down, z forward).
    p2 = np.array(
        [
            [500.0, 0.0, 621.0, 0.0],
            [0.0, 500.0, 187.5, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ]
    )
    r0 = np.eye(3)
    tr = np.array(
        [
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ]
    )

    def flat(matrix: np.ndarray) -> str:
        return " ".join(f"{value:.6f}" for value in matrix.reshape(-1))

    return "\n".join(
        [
            f"P0: {flat(p2)}",
            f"P1: {flat(p2)}",
            f"P2: {flat(p2)}",
            f"P3: {flat(p2)}",
            f"R0_rect: {flat(r0)}",
            f"Tr_velo_to_cam: {flat(tr)}",
            "Tr_imu_to_velo: 1.000000 0.000000 0.000000 0.000000 "
            "0.000000 1.000000 0.000000 0.000000 0.000000 0.000000 1.000000 0.000000",
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "data" / "sample",
    )
    args = parser.parse_args()

    root = args.out
    (root / "image_2").mkdir(parents=True, exist_ok=True)
    (root / "velodyne").mkdir(parents=True, exist_ok=True)
    (root / "calib").mkdir(parents=True, exist_ok=True)

    image_path = root / "image_2" / "000000.png"
    cloud_path = root / "velodyne" / "000000.bin"
    calib_path = root / "calib" / "000000.txt"

    ok, encoded = cv2.imencode(".png", make_image())
    if not ok:
        raise SystemExit(f"failed to encode {image_path}")
    # cv2.imwrite does not reliably handle non-ASCII Windows paths; write bytes.
    image_path.write_bytes(encoded.tobytes())
    make_cloud().tofile(cloud_path)
    calib_path.write_text(make_calibration(), encoding="utf-8")

    print(f"wrote {image_path}")
    print(f"wrote {cloud_path}")
    print(f"wrote {calib_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
