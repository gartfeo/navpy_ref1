import re
import subprocess
import threading

import cv2
import numpy as np

from frame_buffer_writer import FramebufferWriter


class VideoDisplay:
    def __init__(self, display_height=1080):
        self.fb_writer = None
        self.window_name = "Concatenated Frame"
        self.frames = []
        self.fb_width = 1920
        self.fb_height = 1080
        self.display_height = display_height
        self.should_stop = False
        self.frame_ready = threading.Event()
        self.frame_processed = threading.Event()
        self.display_frame = None
        self.show_thread = threading.Thread(target=self.process_and_show)

    def init(self, headless):
        mode = "headless" if headless else "show"
        print(f"Starting video display in headless: {mode} mode...")
        if not headless:
            # Create a named window
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

            # Set the window's property to fullscreen
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            self.show_thread.start()  # Start the display thread
            return

        self.fb_width, self.fb_height = self._get_framebuffer_resolution()
        print("Framebuffer resolution:", self.fb_width, "x", self.fb_height)
        self.fb_writer = FramebufferWriter('/dev/fb0')
        self.show_thread.start()  # Start the display thread

    def show(self, frames):
        self.frames = frames
        self.frame_ready.set()

        if self.fb_writer is None:
            self.frame_processed.wait()
            concatenated_frame = self.display_frame
            if concatenated_frame is not None:
                cv2.imshow(self.window_name, concatenated_frame)
            self.frame_processed.clear()

    def close(self):
        self.should_stop = True
        self.frame_ready.set()
        self.frame_processed.set()
        self.show_thread.join()
        if self.fb_writer is not None:
            self.fb_writer.close()

    def process_and_show(self):
        print("Starting video display thread...")
        while not self.should_stop:
            self.frame_ready.wait()
            if len(self.frames) == 0:
                print("Error: No frame to display.")
                continue
            if any(frame is None for frame in self.frames):
                print("Error: There are empty frames.")
                continue
            concatenated_frame = self.concat_frames_with_aspect_ratio(self.frames, self.display_height)

            if self.fb_writer is not None:
                print("Writing to framebuffer...")
                self.fb_writer.write_to_framebuffer(concatenated_frame)
            else:
                print("Showing frames...")
                if  concatenated_frame is not None:
                    self.display_frame = concatenated_frame
                    self.frame_processed.set()
            self.frame_ready.clear()

    @staticmethod
    def concat_frames_with_aspect_ratio(frames, total_height):
        resized_frames = []
        total_original_height = sum(frame.shape[0] for frame in frames)
        remaining_height = total_height

        for frame in frames[:-1]:
            h, w = frame.shape[:2]
            aspect_ratio = w / h
            proportional_height = int(total_height * (h / total_original_height))
            remaining_height -= proportional_height
            new_width = int(proportional_height * aspect_ratio)
            resized_frame = cv2.resize(frame, (new_width, proportional_height))
            resized_frames.append(resized_frame)

        # Handle last frame separately to account for rounding differences
        last_frame = frames[-1]
        h, w = last_frame.shape[:2]
        aspect_ratio = w / h
        new_width = int(remaining_height * aspect_ratio)
        resized_frame = cv2.resize(last_frame, (new_width, remaining_height))
        resized_frames.append(resized_frame)

        # Pad frames horizontally if necessary and concatenate
        max_width = max(frame.shape[1] for frame in resized_frames)
        padded_frames = [np.hstack([frame, np.zeros((frame.shape[0], max_width - frame.shape[1], 3), dtype=np.uint8)])
                         for frame in resized_frames]
        concatenated_frame = np.vstack(padded_frames)

        # Check if the frame is a numpy array
        if isinstance(concatenated_frame, np.ndarray):
            # Check frame dimensions
            if concatenated_frame.ndim == 3:
                height, width, channels = concatenated_frame.shape
                # Check if the frame is not empty or zero-size
                if height > 0 and width > 0 and channels == 3:
                    return concatenated_frame

        return None

    @staticmethod
    def _get_framebuffer_resolution():
        try:
            # Run the fbset command and capture its output
            output = subprocess.check_output(['fbset', '-fb', '/dev/fb0', '-s'])
            output = output.decode('utf-8')

            # Use regex to find the geometry line
            match = re.search(r'geometry (\d+) (\d+)', output)
            if match:
                width, height = map(int, match.groups())
                return width, height
            else:
                raise ValueError("Unable to parse framebuffer resolution")
        except subprocess.CalledProcessError as e:
            raise RuntimeError("Error running fbset command") from e
