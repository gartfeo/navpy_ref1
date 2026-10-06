import time
import serial

from navpy.args.camera_args import CameraArgs


class CameraZoomControllerVisca:
    """Controls camera zoom via VISCA commands over a serial connection."""
    ZOOM_POSITION_TABLE = [
        '0000',
        '1606',
        '2151',
        '24C8',
        '28F0',
        '2DAC',
        '3418',
        '3727',
        '3B52',
        '4000'
    ]

    def __init__(self, args: CameraArgs, address=0x01):
        """
        :param args: An object (from args.camera_args) with at least `con` (port) and `baud` (baud rate).
        :param address: VISCA device address (default 0x01).
        """
        # Usually 1..7 are valid addresses, so mask just in case
        self._address = address & 0x07
        self._name = args.con
        self._ser = None

        try:
            self._ser = serial.Serial(args.con, args.baud, timeout=1)
            print(f"{self._name}: camera_zoom, initialized (address={hex(self._address)})")
        except serial.SerialException as e:
            print(f"Failed to open serial port {args.con}: {e}")
            raise

    def set_zoom_level(self, level, log=True):
        """
        Sets the zoom level by looking up a predetermined position from ZOOM_POSITION_TABLE.
        :param level: An integer between 1 and len(ZOOM_POSITION_TABLE).
        :param log: Whether to print log messages.
        """
        if not (1 <= level <= len(self.ZOOM_POSITION_TABLE)):
            raise ValueError(
                f"Invalid zoom level: {level}, must be between 1 and {len(self.ZOOM_POSITION_TABLE)}"
            )

        zoom_hex = self.ZOOM_POSITION_TABLE[level - 1]
        # Each nibble of zoom_hex needs to be sent as separate bytes, e.g. "0x00 0x00 0x00 0x00"
        position = f"0{zoom_hex[0]} 0{zoom_hex[1]} 0{zoom_hex[2]} 0{zoom_hex[3]}"

        # First byte of a VISCA command is 0x80 + address
        cmd_start = 0x80 | self._address
        # Example final string might look like: "81 01 04 47 00 00 00 00 FF"
        command_hex = f"{cmd_start:02X} 01 04 47 {position} FF"
        command = bytearray.fromhex(command_hex)

        self._ser.write(command)
        if log:
            print(f"{self._name}: Sent zoom level {level} command.")
        self._wait_for_zoom_completion()

    def get_zoom_level(self):
        """
        Reads the current zoom position from the camera and maps it to the nearest discrete zoom level.
        :return: The integer zoom level (1..N) or None if not available.
        """
        position = self._get_zoom_position()
        if position is not None:
            level = self._get_level_from_position(position)
            print(f"{self._name}: Current zoom position = {position}, mapped to level {level}")
            return level
        else:
            print(f"{self._name}: Failed to get zoom position.")
            return None

    def _get_zoom_position(self):
        """
        Sends an inquiry command for zoom position, then parses the returned data.
        The VISCA inquiry for zoom position is typically: 0x81 0x09 0x04 0x47 0xFF (if address=1).
        :return: The integer (0..0x4000) representing the zoom position or None if error.
        """
        cmd_start = 0x80 | self._address
        command_hex = f"{cmd_start:02X} 09 04 47 FF"
        command = bytearray.fromhex(command_hex)
        self._ser.write(command)

        # Attempt to read response data (could adjust buffer size as needed)
        response = self._ser.read(1024)

        # We look for the sequence: 0x90 0x50, which typically starts the zoom response packet.
        marker = b'\x90\x50'  # 0x90 0x50
        idx = response.find(marker)
        if idx != -1 and len(response) >= idx + 6:
            # Next 4 bytes after 0x90 0x50 are the four nibbles of the zoom position
            zoom_position = (
                    (response[idx + 2] << 12)
                    | (response[idx + 3] << 8)
                    | (response[idx + 4] << 4)
                    | response[idx + 5]
            )
            return zoom_position
        else:
            print("Error reading zoom position (no 0x90 0x50 in response)")
            return None

    def _get_level_from_position(self, position):
        """
        Given a raw integer zoom position, find the nearest level in ZOOM_POSITION_TABLE.
        """
        nearest_level = None
        nearest_diff = float('inf')
        for i, pos_str in enumerate(self.ZOOM_POSITION_TABLE, start=1):
            pos_hex = int(pos_str, 16)
            diff = abs(position - pos_hex)
            if diff == 0:
                return i
            elif diff < nearest_diff:
                nearest_diff = diff
                nearest_level = i
        return nearest_level

    def _wait_for_zoom_completion(self, timeout=5):
        start_time = time.time()
        buffer = bytearray()

        while True:
            chunk = self._ser.read(32)  # read more bytes each iteration
            if chunk:
                buffer.extend(chunk)
                # Look for the command-complete pattern anywhere in the buffer
                if b'\x90\x51\xff' in buffer:
                    print(f"{self._name}: Zoom completed.")
                    return
            if time.time() - start_time > timeout:
                print(f"{self._name}: Zoom completion timed out.")
                return
            time.sleep(0.02)

    def close(self):
        """Close the serial connection."""
        if self._ser and self._ser.is_open:
            self._ser.close()
            print(f"{self._name}: Serial port closed.")
