from gpiozero import Button
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import ImageFont
import subprocess


def get_local_ip():
    try:
        result = subprocess.check_output(['hostname', '-I']).decode().strip()
        return f"IP: {result.split()[0]}"  # Get first IP address
    except Exception:
        return "IP Error"

# OLED Controller
class OLEDController:
    def __init__(self, port=1, address=0x3C, width=128, height=64):
        serial = i2c(port=port, address=address)
        self.device = ssd1306(serial, width=width, height=height)
        self.font = ImageFont.load_default()

    def display_info(self, lines):
        with canvas(self.device) as draw:
            for i, line in enumerate(lines):
                draw.text((0, i * 16), line, fill="white", font=self.font)

    def clear(self):
        self.device.clear()

# Create instances
oled = OLEDController()
button = Button(22)


