import time
import argparse

# If your project structure has these modules,
# adjust the import paths as needed.
from navpy.args.camera_controller_args import CameraControllerArgs
from tools.cam.zoom.camera_zoom_controller_visca import CameraZoomControllerVisca

def set_and_check_zoom_level(camera: CameraZoomControllerVisca, level: int, wait: float = 1.0):
    """
    Helper function that:
      - sets the zoom level,
      - waits a bit (optional),
      - then gets/prints the resulting zoom level.
    """
    camera.set_zoom_level(level, log=False)
    # If you prefer an explicit delay to allow the camera time to move:
    time.sleep(wait)
    zoom_level = camera.get_zoom_level()
    print(f"Set zoom level to {level}, got {zoom_level}")

def main():
    """
    Example main function that:
      1) Parses command-line arguments for two potential cameras,
      2) Initializes camera #1 on COM16 at 9600 baud,
      3) Runs through a few test zoom operations,
      4) Closes the connection.
    """
    # Defaults you can change:
    default_port = 'COM10'   # e.g., 'COM8' on Windows, '/dev/ttyUSB0' on Linux
    default_baud = 9600

    # Build an argument parser (assuming CameraControllerArgs is part of your existing code)
    parser = argparse.ArgumentParser(description="Camera Zoom Run Test")
    # The following lines add placeholders for two cameras
    CameraControllerArgs.add_args(parser, 0)
    CameraControllerArgs.add_args(parser, 1)

    # Parse any command-line arguments passed in
    args = parser.parse_args()

    # Create a CameraControllerArgs object for camera #0
    cam1_args = CameraControllerArgs(args, 0)
    # Override with specific port/baud if desired
    cam1_args.con = default_port
    cam1_args.baud = default_baud

    # Initialize the camera (address=0x01 is typical for a single VISCA camera)
    camera = CameraZoomControllerVisca(cam1_args, address=0x01)

    try:
        camera.set_zoom_level(level=2, log=False)
        # # Example 1: Set zoom level 1 and verify
        # set_and_check_zoom_level(camera, 1, 2)
        #
        # # Example 2: Iterate from zoom level 1..10 and check each
        # for i in range(1, 11):
        #     set_and_check_zoom_level(camera, i, 1)
        #
        # # Example 3: Query the camera's current zoom level, then re-set it
        # current = camera.get_zoom_level()
        # print(f"Current zoom level is {current}; re-setting the same level...")
        # set_and_check_zoom_level(camera, current or 1, 2)
        #
        # # Example 4: Back to level 1
        # set_and_check_zoom_level(camera, 1, 2)

    finally:
        # Always close the connection when done
        camera.close()

if __name__ == "__main__":
    main()
