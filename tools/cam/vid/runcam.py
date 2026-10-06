import argparse

from video_display import VideoDisplay
from video_processor import VideoProcessor

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera Video Processing')
    parser.add_argument('-s', '--sources', nargs='*', default=['/dev/video0', '/dev/video/2'],
                        help='List of video sources.')
    parser.add_argument('-dp', '--display-height', type=int, default=1080, help='Height for the display window.')
    parser.add_argument('-sp', '--save-path', type=str, default='vid', help='Path for saving videos.')
    parser.add_argument('-hl', '--headless', action='store_true', help='Run in headless mode.')

    args = parser.parse_args()

    display = VideoDisplay(args.display_height)
    processor = VideoProcessor(display)
    try:
        display.init(args.headless)

        sources = [int(source) if source.isnumeric() else source for source in args.sources]
        if processor.init(sources, args.save_path):
            processor.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected.")
    finally:
        display.close()
        processor.close()
