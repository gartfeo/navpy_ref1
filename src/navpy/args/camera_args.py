import sys


class CameraArgs:
    def __init__(self, args, index):
        self.con = getattr(args, f'camera_con{index}')
        self.baud = getattr(args, f'camera_baud{index}')
        self.zoom_channel = getattr(args, f'zoom_channel{index}')

    @staticmethod
    def add_args(parser, index):
        if sys.platform == 'linux':
            if index == 0:
                default_c_conn = ''
                # default_c_conn = '/dev/ttyUSB0'
            else:
                default_c_conn = ''
                # default_c_conn = '/dev/ttyUSB1'
        else:
            if index == 0:
                default_c_conn = ''
                # default_c_conn = 'COM11'
            else:
                default_c_conn = ''
                # default_c_conn = 'COM14'

        camera_args = parser.add_argument_group("Camera arguments")
        camera_args.add_argument(f'-cc{index}', f'--camera-con{index}', type=str, default=default_c_conn,
                                 help=f'Serial port (COM11, /dev/ttyUSB0) for VISCA commands (default: {default_c_conn})')
        camera_args.add_argument(f'-cb{index}', f'--camera-baud{index}', type=int, default=9600,
                                 help='Baudrate for camera (default: 9600)')
        camera_args.add_argument(f'-zc{index}', f'--zoom-channel{index}', type=int, default=13,
                                 help='RC channel number for zoom control (default: 13)')
