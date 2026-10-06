import os

import cv2
import numpy as np

from tools.cam.zoom.camera_zoom_controller_visca import CameraZoomControllerVisca

# Chessboard dimensions (inner corners)
rows = 6
cols = 9
chessboard_dims = (cols, rows)

# Termination criteria for subpixel corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ..., (5,8,0)
objp = np.zeros((rows * cols, 3), np.float32)
objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane


def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder, filename))
        if img is not None:
            images.append(img)
    return images


def main():
    zoom_level = 2
    camera = CameraZoomControllerVisca('COM8', 9600)
    camera.set_zoom_level(zoom_level)

    base_dir = f'C:\\repos\\aas\\navpy\\camera\\images\\zoom_{zoom_level}'
    if not os.path.exists(base_dir):
        os.mkdir(base_dir)
    cap = cv2.VideoCapture(1)
    # frames = load_images_from_folder('C:\\repos\\aas\\navpy\\camera\\novoxy_10')
    try:

        img_id = 1
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, chessboard_dims, None)

            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # Draw and display the corners
                cv2.drawChessboardCorners(frame, chessboard_dims, corners2, ret)

            cv2.imshow('Camera Calibration', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif ret and key == ord('t'):
                img_dir = f'{base_dir}\\image_{img_id}.png'
                if not cv2.imwrite(img_dir, frame):
                    print('Could not save image')
                else:
                    print(f'saved image in: {img_dir}')
                    img_id = img_id + 1
                objpoints.append(objp)
                imgpoints.append(corners2)
    finally:
        # Perform calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        print("Camera matrix: \n", mtx)
        print("Distortion coefficients: \n", dist)

        # cap.release()
        # cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
