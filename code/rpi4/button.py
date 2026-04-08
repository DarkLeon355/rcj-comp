import os
import subprocess
import RPi.GPIO as GPIO
import time
import signal
import sys

BUTTON_PIN = 26
# Ensure we have absolute paths
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_ON = os.path.join(BASE_DIR, "led_on.sh")
SCRIPT_OFF = os.path.join(BASE_DIR, "led_off.sh")
PYTHON_SCRIPT = os.path.join(BASE_DIR, "main.py")

def handle_led(state):
    script = SCRIPT_ON if state == "on" else SCRIPT_OFF
    try:
        # Use Popen to keep it non-blocking if needed, 
        # or stick to run if the scripts are near-instant.
        subprocess.run(["sudo", "/bin/bash", script], check=True)
    except Exception:
        pass


def wait_for_button_press():
    try:
        GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING, bouncetime=250)
        return True
    except RuntimeError:
        # Fallback for environments where edge interrupts are unavailable.
        while GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            time.sleep(0.01)
        time.sleep(0.05)
        return GPIO.input(BUTTON_PIN) == GPIO.LOW

def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    # Added PUD_UP and pull_up_down logic
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    led_state = "off"
    python_proc = None

    try:
        while True:
            if not wait_for_button_press():
                continue

            if python_proc and python_proc.poll() is None:
                led_state = "off"
                handle_led(led_state)
                python_proc.send_signal(signal.SIGINT)
                try:
                    python_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    python_proc.terminate()
                    python_proc.wait(timeout=5)
                python_proc = None
            else:
                led_state = "on"
                handle_led(led_state)
                python_proc = subprocess.Popen([sys.executable, PYTHON_SCRIPT])

            # Wait for button release to avoid immediate re-triggering
            while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        if python_proc and python_proc.poll() is None:
            python_proc.terminate()
        GPIO.cleanup()

if __name__ == "__main__":
    main()