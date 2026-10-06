import datetime
import os
import threading
from pathlib import Path

import cv2


class VideoProcessor:
    def __init__(self, video_display):
        self.caps = []
        self.outs = []
        self.save_threads = []
        self.frames = [None, None]
        self.stop = False
        self.video_display = video_display
        self.frame_ready = threading.Event()

    def init(self, sources, path):
        path_to_save = Path.home() / path
        if not os.path.exists(str(path_to_save)):
            print("Creating video saving directory...")
            os.makedirs(str(path_to_save))  # Create directory if it doesn't exist

        for index in range(len(sources)):
            source = sources[index]
            cap, out = self.create_cap_out(index, source, path_to_save)
            if not (cap.isOpened() and out.isOpened()):
                print(f'Error: Could not open video stream for {source}.')
                exit()
            self.caps.append(cap)
            self.outs.append(out)

        # Check if cameras opened successfully
        if len(self.caps) == 0:
            print("Error: Could not open video streams.")
            exit()

        print("Creating video saving threads...")
        for index in range(len(self.caps)):
            self.save_threads.append(threading.Thread(target=self.save_video, args=[index]))
        return True

    def run(self):
        # for thread in self.save_threads:
        # thread.start()

        print("Starting video streams...")
        while not self.stop:
            self.save_video(0)
            self.save_video(1)
            self.frame_ready.wait()

            if all(frame is not None for frame in self.frames):
                self.video_display.show(self.frames)
            else:
                print("Warning: No frames to display.")
            self.frame_ready.clear()

            # Break the loop with the 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def save_video(self, index):
        print(f"Starting video saving thread for video stream {index}...")

        # while not self.stop:
        ret, frame = self.caps[index].read()
        if not ret:
            print(f'Warning: Could not read frame from video stream {index}.')
            return
        self.outs[index].write(frame)
        self.frames[index] = frame
        if self.frames[index] is None:
            print(f"Warning: No frame to display. Index {index}")
        self.frame_ready.set()

    def close(self):
        self.stop = True

        try:
            print("Closing all windows...")
            for save_thread in self.save_threads:
                save_thread.join()
        except Exception as e:
            print(f"Error in closing all windows: {e}")

        try:
            for cap in self.caps:
                print(f"{cap} Releasing video capture objects...")
                cap.release()
        except Exception as e:
            print(f"Error in releasing video capture objects: {e}")

        try:
            for out in self.outs:
                print(f"{out} Releasing video writer objects...")
                out.release()
        except Exception as e:
            print(f"Error in releasing video writer objects: {e}")
        cv2.destroyAllWindows()

    @staticmethod
    def create_cap_out(index, source, path_to_save):
        print("Starting video recording streams...")

        # Use the integer index for the camera
        cap = cv2.VideoCapture(source)  # Convert source to integer

        # Set properties for capture
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps == 0:  # Fallback if fps couldn't be determined
            fps = 25

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print(f"Fps: {fps}")

        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        out = cv2.VideoWriter(
            f"{path_to_save}/vid{index}_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.avi",
            fourcc,
            fps,
            (width, height))

        return cap, out
