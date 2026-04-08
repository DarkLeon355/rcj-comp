import time
import board
import neopixel

# -------------------------
# Configuration
# -------------------------
LED_PIN = board.D18       # GPIO pin where the NeoPixel data line is connected
NUM_PIXELS = 8            # Number of LEDs you want to control
BRIGHTNESS = 0.5          # Brightness (0 to 1, 1 is full brightness)

# -------------------------
# Initialize NeoPixel strip
# -------------------------
pixels = neopixel.NeoPixel(LED_PIN, NUM_PIXELS, brightness=BRIGHTNESS, auto_write=True)

# -------------------------
# Turn all LEDs white
# -------------------------
pixels.fill((0, 0, 0))  # RGB: white
print(f"All {NUM_PIXELS} pixels should now be off!")

