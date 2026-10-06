import cv2


def test_camera(index):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if cap.isOpened():
        print(f"Camera at index {index} is available.")
        cap.release()
    else:
        print(f"Camera at index {index} is not available.")


for i in range(5):  # Test the first 5 indices
    test_camera(i)
