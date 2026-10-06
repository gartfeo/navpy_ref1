import argparse
import cv2

from video_processor import VideoProcessor


def main(params):
    # Capture video
    print("Starting video recording streams...")
    cap, out = VideoProcessor.create_cap_out(0, params.source, params.path)

    if cap is None or out is None:
        print("Failed to initialize video capture or output stream")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            out.write(frame)
            # Break the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        print("Shutting down...")
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        print("Done")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera Video Processing')
    parser.add_argument('-s', '--source', type=str, default='0', help='Index for the first video source.')
    parser.add_argument('-p', '--path', type=str, default='vid', help='Path for saving videos.')
    args = parser.parse_args()
    main(args)
