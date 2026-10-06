import serial
from serial import rs485


class CameraZoomControllerPelcoD(object):
    def __init__(self, conn, baud):
        self._previous_zoom_command = None
        self._zoom_in_command = bytes([0xFF, 0x01, 0x00, 0x20, 0x00, 0x00, 0x21])
        self._zoom_out_command = bytes([0xFF, 0x01, 0x00, 0x40, 0x00, 0x00, 0x41])
        self._zoom_stop_command = bytes([0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01])

        self._ser = serial.Serial(conn, baud)
        self._ser.rs485_mode = rs485.RS485Settings(rts_level_for_tx=True, rts_level_for_rx=False, delay_before_tx=None,
                                                   delay_before_rx=None)
        print('camera_zoom, initialized')

    def zoom_in(self):
        self.send_zoom_command(self._zoom_in_command, 'zoom_in')

    def zoom_out(self):
        self.send_zoom_command(self._zoom_out_command, 'zoom_out')

    def zoom_stop(self):
        self.send_zoom_command(self._zoom_stop_command, 'stop zoom')

    def send_zoom_command(self, zoom_command, display_msg):
        # Send zoom command over serial port
        if zoom_command == self._zoom_stop_command and self._previous_zoom_command == self._zoom_stop_command:
            return
        self._ser.write(zoom_command)
        print(display_msg)
        self._previous_zoom_command = zoom_command

    def close(self):
        self._ser.close()
