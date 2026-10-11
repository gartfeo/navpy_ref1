#!/usr/bin/env python

import argparse
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

CALIBRATION_DIR = Path(__file__).resolve().parent


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


IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")


def _list_images(image_dir: Path) -> list[str]:
    return sorted(
        str(p) for p in image_dir.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES
    )


def _collect_detections(images, checkerboard, objp, criteria, show=False):
    """Detect board corners in every image of the calibration frame size.

    calibrateCamera needs one image size; the most common size in the set is
    the calibration frame and images of any other size (e.g. cropped captures)
    are skipped, not fitted.
    """
    objpoints, imgpoints = [], []
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

    loaded = []
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Skipping unreadable image: {fname}")
            continue
        loaded.append((fname, img))
    if not loaded:
        return objpoints, imgpoints, None

    img_shape = Counter(img.shape[1::-1] for _, img in loaded).most_common(1)[0][0]

    for fname, img in loaded:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]
        if shape != img_shape:
            print(f"Skipping {Path(fname).name}: size {shape} != calibration size {img_shape}")
            continue

        ret, corners = cv2.findChessboardCorners(gray, checkerboard, flags)
        if not ret and hasattr(cv2, "findChessboardCornersSB"):
            ret, corners = cv2.findChessboardCornersSB(gray, checkerboard, None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)

            if show:
                overlay = cv2.drawChessboardCorners(img, checkerboard, corners2, ret)
                cv2.imshow("calib", overlay)
                cv2.waitKey(50)
        else:
            print(f"No corners: {Path(fname).name}")

    return objpoints, imgpoints, img_shape


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate camera from chessboard images.")
    parser.add_argument(
        "--image-dir",
        default=str(CALIBRATION_DIR / "novoxy_10" / "zoom1"),
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
        default=str(CALIBRATION_DIR / "novoxy_10" / "zoom1" / "intrinsics.json"),
        help="Output JSON file for intrinsics.",
    )
    parser.add_argument(
        "--camera-name",
        default="Novoxy_10",
        help="Camera key written to the output JSON (default: Novoxy_10).",
    )
    parser.add_argument(
        "--zoom",
        default="1",
        help="Zoom key written to the output JSON (default: 1).",
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

    images = _list_images(Path(args.image_dir))
    if not images:
        raise SystemExit(f"No images found in {args.image_dir}")

    objpoints, imgpoints, img_shape = _collect_detections(
        images, checkerboard, objp, criteria, show=args.show
    )

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
            args.camera_name: {
                "image_width": int(w),
                "image_height": int(h),
                "zooms": {
                    str(args.zoom): {
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
