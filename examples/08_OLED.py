#!/usr/bin/env/python
# File name   : 08_OLED.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date        : 2025/03/7
import board
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import datetime
import time

# Create an I2C object
i2c = busio.I2C(board.SCL, board.SDA)

# Create an SSD1306 OLED device object with a screen resolution of 128x64
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

# Load the font
font = ImageFont.load_default()

def draw_text_with_wrap(draw, text, x, y, font, fill, max_width):
    lines = []
    current_line = ""
    for word in text.split():
        test_line = current_line + word + " "
        # Use the textlength method to get the text width
        test_width = draw.textlength(test_line, font=font)
        if test_width <= max_width:
            current_line = test_line
        else:
            lines.append(current_line.rstrip())
            current_line = word + " "
    if current_line:
        lines.append(current_line.rstrip())

    for line in lines:
        draw.text((x, y), line, font=font, fill=fill)
        y += font.getsize(line)[1]

try:
    while True:
        # Clear the screen
        oled.fill(0)

        # Create a blank image
        image = Image.new('1', (oled.width, oled.height))

        # Create a drawing object
        draw = ImageDraw.Draw(image)

        # Get the current time
        now = datetime.datetime.now()
        current_time = now.strftime("%Y-%m-%d %H:%M:%S")
        display_text = f"Time: {current_time}"

        # Draw the time on the image with word wrap support
        draw_text_with_wrap(draw, display_text, 0, 0, font, 255, oled.width)

        # Display the image on the OLED screen
        oled.image(image)
        oled.show()

        # Pause for 1 second
        time.sleep(1)

except KeyboardInterrupt:
    # Clear the screen
    oled.fill(0)
    oled.show()
