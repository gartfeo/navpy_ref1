import array
import struct
import fcntl
import cv2
import numpy as np


class FramebufferWriter:
    FBIOGET_VSCREENINFO = 0x4600

    def __init__(self, fb_device):
        self.fb = open(fb_device, 'r+b')
        self.fb_info = self.get_framebuffer_info()
        print(self.fb_info)
        self.fb_width = self.fb_info['xres']
        self.fb_height = self.fb_info['yres']

    def display_color_gradient(self):
        # Create an image with a horizontal color gradient
        gradient = np.zeros((self.fb_height, self.fb_width, 3), dtype=np.uint8)

        # Calculate the color change per pixel
        for i in range(self.fb_width):
            red = int((255 * i) / self.fb_width)  # Red value varies from 0 to 255
            green = 255 - red  # Green value is the inverse of red
            blue = 128  # Constant blue value for simplicity

            gradient[:, i] = [blue, green, red]  # Set the color for each column

        # Write the gradient to the framebuffer
        self.write_to_framebuffer(gradient)

    def get_framebuffer_info(self):
        # Create a buffer of the correct size
        buffer = array.array('B', [0] * 144)  # Adjust the size as necessary

        # Perform the ioctl call
        fcntl.ioctl(self.fb, self.FBIOGET_VSCREENINFO, buffer)

        # Unpack the buffer
        # Adjust the format string to match the complete fb_var_screeninfo structure
        # This is a hypothetical example; the actual format depends on your system's structure definition
        format_str = '8I12I16I'
        vsinfo = struct.unpack(format_str, buffer)

        return {
            'xres': vsinfo[0],
            'yres': vsinfo[1],
            'bits_per_pixel': vsinfo[6],
            'red_offset': vsinfo[8],
            'red_length': vsinfo[9],
            'green_offset': vsinfo[10],
            'green_length': vsinfo[11],
            'blue_offset': vsinfo[12],
            'blue_length': vsinfo[13],
            'transp_offset': vsinfo[14],  # Transparency channel
            'transp_length': vsinfo[15],
        }

    def write_to_framebuffer(self, frame):
        # Calculate aspect ratio of the input frame
        frame_height, frame_width = frame.shape[:2]
        aspect_ratio = frame_width / frame_height

        # Calculate the new dimensions to fit the frame into the framebuffer size
        new_height = int(self.fb_width / aspect_ratio)
        if new_height > self.fb_height:
            new_height = self.fb_height
            new_width = int(self.fb_height * aspect_ratio)
        else:
            new_width = self.fb_width

        frame_converted = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Resize the frame to new dimensions
        frame_resized = cv2.resize(frame_converted, (new_width, new_height))

        # Create a black canvas of framebuffer size
        canvas = np.zeros((self.fb_height, self.fb_width, 3), dtype=np.uint8)

        # Calculate centering position
        x_offset = (self.fb_width - new_width) // 2
        y_offset = (self.fb_height - new_height) // 2

        # Place the resized frame onto the canvas
        canvas[y_offset:y_offset + new_height, x_offset:x_offset + new_width] = frame_resized

        # Convert the canvas to RGB 565
        frame_rgb565 = np.zeros((self.fb_height, self.fb_width, 2), dtype=np.uint8)
        frame_rgb565[..., 1] = (canvas[..., 0] & 0xF8) | (canvas[..., 1] >> 5)
        frame_rgb565[..., 0] = ((canvas[..., 1] & 0x1C) << 3) | (canvas[..., 2] >> 3)

        # Flatten the frame data for writing
        frame_data = frame_rgb565.flatten()

        # Write to framebuffer
        self.fb.write(frame_data.tobytes())
        self.fb.flush()
        self.fb.seek(0)

    def convert_to_framebuffer_format(self, canvas):
        # Extract bit lengths and offsets from fb_info
        red_length = self.fb_info['red_length']
        green_length = self.fb_info['green_length']
        blue_length = self.fb_info['blue_length']
        red_offset = self.fb_info['red_offset']
        green_offset = self.fb_info['green_offset']
        blue_offset = self.fb_info['blue_offset']

        # Calculate shift values for each channel
        red_shift = red_offset
        green_shift = green_offset
        blue_shift = blue_offset

        # Shift and mask each channel
        red = (canvas[..., 2] >> (8 - red_length)) << red_shift
        green = (canvas[..., 1] >> (8 - green_length)) << green_shift
        blue = (canvas[..., 0] >> (8 - blue_length)) << blue_shift

        # Combine the channels into a 16-bit format
        frame_buffer_format = np.zeros((self.fb_height, self.fb_width, 2), dtype=np.uint8)
        frame_buffer_format[..., 0] = np.bitwise_or(blue, green & 0xE0)  # Combining blue and lower 3 bits of green
        frame_buffer_format[..., 1] = np.bitwise_or(red, (green & 0x1C) << 3)  # Combining red and upper 3 bits of green

        return frame_buffer_format

    def close(self):
        self.fb.close()


if __name__ == '__main__':
    fb_writer = FramebufferWriter('/dev/fb0')
    fb_writer.display_color_gradient()
    fb_writer.close()
