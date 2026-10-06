import datetime

import cv2
import argparse


def capture_video(source, width, height, fps):
    # Start capturing video from the webcam
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)

    # Set properties for capture
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # Get the actual frame rate of the webcam
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:
        fps = 25  # Assume a default if it cannot be determined
    print(f"FPS: {fps}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:  # Fallback if fps couldn't be determined
        fps = 25

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Fps: {fps}")

    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter(
        f"/home/pf/vid/vid0_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.avi",
        fourcc,
        fps,
        (width, height))

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            # Write the frame into the file 'output.avi'
            out.write(frame)

            # Display the resulting frame
            cv2.imshow('frame', frame)

            # Press 'q' on the keyboard to stop recording
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    # Release everything when the job is finished
    cap.release()
    out.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Capture video from a camera source.")
    parser.add_argument('--source', type=str, default='0',
                        help="Camera source, e.g., 0 for the default camera or '/dev/video0' for a specific device path.")
    parser.add_argument('--width', type=int, default=1920, help="Width of the video.")
    parser.add_argument('--height', type=int, default=1080, help="Height of the video.")
    parser.add_argument('--fps', type=int, default=25, help="FPS of the video.")
    args = parser.parse_args()

    # Convert source to integer if it's purely numeric
    source = int(args.source) if args.source.isnumeric() else args.source
    capture_video(source, args.width, args.height, args.fps)


if __name__ == "__main__":
    main()
