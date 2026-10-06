import serial


def send_command(serial_port, cmd):
    serial_port.write(bytearray(cmd))


def read_response(serial_port):
    response = []
    while True:
        byte = serial_port.read(1)
        if byte:
            response.append(int.from_bytes(byte, 'big'))
            if len(response) >= 7:
                break
        else:
            break
    return response


def get_zoom_position(serial_port):
    zoom_inquiry_command = [0x81, 0x09, 0x04, 0x47, 0xFF]
    send_command(serial_port, zoom_inquiry_command)
    response = read_response(serial_port)

    if len(response) == 7 and response[:3] == [0x90, 0x50, 0x00]:
        zoom_position = (response[3] << 12) + (response[4] << 8) + (response[5] << 4) + response[6]
        return zoom_position
    else:
        return None


# Replace 'COM1' with your PTZ run_tests's serial port
serial_port = serial.Serial('COM1', 9600)

zoom_position = get_zoom_position(serial_port)
if zoom_position is not None:
    print(f"Current zoom position: {zoom_position}")
else:
    print("Failed to get zoom position")

serial_port.close()
