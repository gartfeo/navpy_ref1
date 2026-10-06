#!/usr/bin/env python

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np


def _parse_pattern(pattern: str) -> tuple[int, int]:
    parts = pattern.lower().split("x")
    if len(parts) != 2:
        raise ValueError("Pattern must look like COLSxROWS (e.g., 8x6).")
    cols = int(parts[0])
    rows = int(parts[1])
    return cols, rows


def _build_objp(cols: int, rows: int, square_size_mm: float) -> np.ndarray:
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_size_mm)
    return objp


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate camera from chessboard images.")
    parser.add_argument(
        "--image-dir",
        default=r"C:\repos\navpy\tools\cam\calibration\novoxy_10\zoom1",
        help="Directory with calibration images.",
    )
    parser.add_argument(
        "--pattern",
        default="8x6",
        help="Checkerboard inner corners as COLSxROWS (default: 8x6).",
    )
    parser.add_argument(
        "--square-size-mm",
        type=float,
        default=30.0,
        help="Chessboard square size in mm (default: 30).",
    )
    parser.add_argument(
        "--output-json",
        default=r"C:\repos\navpy\tools\cam\calibration\novoxy_10\zoom1\intrinsics.json",
        help="Output JSON file for intrinsics.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show detection overlays while processing.",
    )
    args = parser.parse_args()

    cols, rows = _parse_pattern(args.pattern)
    checkerboard = (cols, rows)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objpoints = []
    imgpoints = []
    objp = _build_objp(cols, rows, args.square_size_mm)

    images = sorted(glob.glob(str(Path(args.image_dir) / "*.jpg")))
    if not images:
        raise SystemExit(f"No images found in {args.image_dir}")

    img_shape = None
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Skipping unreadable image: {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(gray, checkerboard, flags)
        if not ret and hasattr(cv2, "findChessboardCornersSB"):
            ret, corners = cv2.findChessboardCornersSB(gray, checkerboard, None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)

            if args.show:
                overlay = cv2.drawChessboardCorners(img, checkerboard, corners2, ret)
                cv2.imshow("calib", overlay)
                cv2.waitKey(50)
        else:
            print(f"No corners: {Path(fname).name}")

    if args.show:
        cv2.destroyAllWindows()

    print(f"Usable images: {len(imgpoints)} / {len(images)}")
    if len(imgpoints) < 5:
        raise SystemExit("Not enough detections to calibrate (need ~5+).")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)

    print("Camera matrix :")
    print(mtx)
    print("dist :")
    print(dist)
    print(f"RMS: {ret}")

    mean_error = 0.0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    mean_error /= len(objpoints)
    print(f"Reprojection error: {mean_error}")

    output_path = Path(args.output_json)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    h, w = img_shape[1], img_shape[0]

    intrinsics = {
        "cameras": {
            "Novoxy_10": {
                "image_width": int(w),
                "image_height": int(h),
                "zooms": {
                    "1": {
                        "fx": float(mtx[0, 0]),
                        "fy": float(mtx[1, 1]),
                        "cx": float(mtx[0, 2]),
                        "cy": float(mtx[1, 2]),
                        "skew": float(mtx[0, 1]) / float(mtx[0, 0]) if mtx[0, 0] != 0 else 0.0,
                        "dist": [float(x) for x in dist.ravel().tolist()],
                    }
                },
            }
        }
    }

    output_path.write_text(json.dumps(intrinsics, indent=2))
    print(f"Wrote intrinsics JSON: {output_path}")


if __name__ == "__main__":
    main()
