import os
import subprocess
import RPi.GPIO as GPIO
import time
import signal

BUTTON_PIN = 26
SCRIPT_ON = "led_on.sh"
SCRIPT_OFF = "led_off.sh"
PYTHON_SCRIPT = "main.py"
DOUBLE_PRESS_WINDOW = 0.6

def handle_one_press(led_state):
    if led_state == "off":
        script_path = os.path.join(os.path.dirname(__file__), SCRIPT_ON)
    else:
        script_path = os.path.join(os.path.dirname(__file__), SCRIPT_OFF)
    subprocess.run(["sudo", "/bin/bash", script_path], check=True)

def start_python_program():
    script_path = os.path.join(os.path.dirname(__file__), PYTHON_SCRIPT)
    return subprocess.Popen(["python3", script_path])

def stop_python_program(proc):
    # Send SIGINT so main.py can run its KeyboardInterrupt/finally cleanup path.
    proc.send_signal(signal.SIGINT)
    proc.wait()

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    led_state = "off"
    python_proc = None
    pending_single_press_at = None

    print("Warte auf klick an GPIO 26")
    try:
        while True:
            now = time.time()

            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                    time.sleep(0.01)
                print("Kurzer klick erkannt")

                if pending_single_press_at is not None and now - pending_single_press_at <= DOUBLE_PRESS_WINDOW:
                    if python_proc and python_proc.poll() is None:
                        print("Doppelklick beende Python Programm")
                        stop_python_program(python_proc)
                        python_proc = None
                    else:
                        print("Doppelklick starte Python Programm")
                        python_proc = start_python_program()
                    pending_single_press_at = None
                else:
                    pending_single_press_at = now

                time.sleep(0.1)

            if pending_single_press_at is not None and now - pending_single_press_at > DOUBLE_PRESS_WINDOW:
                print("Einzelklick bestätigt")
                handle_one_press(led_state)
                led_state = "on" if led_state == "off" else "off"
                pending_single_press_at = None

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if python_proc and python_proc.poll() is None:
            stop_python_program(python_proc)

        GPIO.cleanup()

if __name__ == "__main__":
    main()