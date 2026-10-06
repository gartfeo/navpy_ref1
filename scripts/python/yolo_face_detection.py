from pathlib import Path

import cv2
from ultralytics import YOLO

# Folder that contains *this* Python file
current_path = Path(__file__).resolve().parents[2]

MODEL = (current_path / '.models' / 'yolov8n-face-lindevs.pt').resolve()
print(MODEL)
CONF = 0.35
IMG_SZ = 640
CAM_INDEX = 1

model = YOLO(MODEL)

# DSHOW is usually the most reliable backend on Windows
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

if not cap.isOpened():
    raise RuntimeError("Camera not opened. Try CAM_INDEX=1 or 2.")

cv2.namedWindow("Face Detection", cv2.WINDOW_NORMAL)

print("Press ESC to exit.")
while True:
    ok, frame = cap.read()
    if not ok:
        print("Frame read failed")
        break

    res = model.predict(frame, imgsz=IMG_SZ, conf=CONF, verbose=False)[0]

    if res.boxes is not None:
        for b in res.boxes:
            x1, y1, x2, y2 = b.xyxy[0].cpu().numpy()
            conf = float(b.conf[0].cpu().numpy())

            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{conf:.2f}", (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Face Detection", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
