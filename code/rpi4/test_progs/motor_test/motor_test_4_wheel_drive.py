import motors
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)



motors = motors.Motors()


while True:
    motors.forward(100)

