import RPi.GPIO as GPIO
import time
from typing import Optional
 

class HC_SR04:
    """Ultrasonic distance sensor driver for HC-SR04 using RPi.GPIO."""

    def __init__(self, trigger_pin: int, echo_pin: int) -> None:
        """Initialize GPIO pins for trigger and echo.

        Args:
            trigger_pin: BCM GPIO pin used for trigger output.
            echo_pin: BCM GPIO pin used for echo input.
        """

        #─────────────────────────────────────────────────────────────
        # SETUP:
        #─────────────────────────────────────────────────────────────
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin

        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        #─────────────────────────────────────────────────────────────
 
    def measure(self) -> Optional[float]:
        """Measure distance in centimeters.

        Returns:
            Measured distance in cm, or None when a timeout occurs.
        """
        GPIO.output(self.trigger_pin, False)
        time.sleep(0.01)
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, False)

        start = time.time()
        stop = time.time()

        timeout_start = time.time()
        while GPIO.input(self.echo_pin) == 0:
            start = time.time()
            if (start - timeout_start) > 0.1:
                return None

        timeout_start = time.time()
        while GPIO.input(self.echo_pin) == 1:
            stop = time.time()
            if (stop - timeout_start) > 0.1:
                return None

        time_elapsed = stop - start
        dist = (time_elapsed * 34300) / 2

        return dist
 
if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    try:
        sensor = HC_SR04(trigger_pin=4, echo_pin=25)
        while True:
            dist = sensor.measure()
            if dist is not None:
                print(f"Distance = {dist:.1f} cm")
            else:
                print("Measurement failed!")
            time.sleep(1)
 
        # Beim Abbruch durch STRG+C resetten
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user")
        GPIO.cleanup()