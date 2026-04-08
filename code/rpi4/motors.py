import RPi.GPIO as GPIO
import time
import gyroscope
import numpy as np


class Motors:
    def __init__(self, gyro=None) -> None:
        """Initialize motor GPIO pins, PWM channels, and gyro-based turn settings.

        Args:
            gyro: Optional gyroscope instance that provides get_gyro_z().
        """
        self.gyro = gyro

        #─────────────────────────────────────────────────────────────
        # DEFINITIONS:
        #─────────────────────────────────────────────────────────────
        # Motor pins
        self.IN1 = 24
        self.IN2 = 23
        self.ENA = 12  # PWM for motor 1
        self.IN3 = 17
        self.IN4 = 27
        self.ENB = 13  # PWM for motor 2

        # pwm frequency in Hz
        self.pwm_freq = 5000

        # Gyro parameters
        self.GYRO_THRESHOLD = 2  # degrees/sec, ignore small jitter
        self.TURN_90_ANGLE = 85  # degrees, target angle for 90 degree turns
        self.TURN_180_ANGLE = 170 # degrees, target angle for 180 degree turns
        self.TURN_45_ANGLE = 40 # degrees, target angle for 45 degree turns

        # Motor speed parameters
        self.turn_speed = 100


        #─────────────────────────────────────────────────────────────

        #─────────────────────────────────────────────────────────────
        # SETUP:
        #─────────────────────────────────────────────────────────────
        # Setup GPIO
        for pin in [self.IN1, self.IN2, self.ENA, self.IN3, self.IN4, self.ENB]:
            GPIO.setup(pin, GPIO.OUT)

        # Setup PWM at 1kHz
        self.pwmA = GPIO.PWM(self.ENA, self.pwm_freq)
        self.pwmB = GPIO.PWM(self.ENB, self.pwm_freq)

        # Start PWM at 0% duty
        self.pwmA.start(0)
        self.pwmB.start(0)
        #─────────────────────────────────────────────────────────────

    def stop(self) -> None:
        """Stop both motors by setting PWM duty cycles to zero."""
        self.pwmA.ChangeDutyCycle(0)
        self.pwmB.ChangeDutyCycle(0)

    def forward(self, speed: int) -> None:
        """Drive both motors forward with the given duty cycle.

        Args:
            speed: PWM duty cycle in percent (0-100).
        """
        self.pwmA.ChangeDutyCycle(speed)
        self.pwmB.ChangeDutyCycle(speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

    def backward(self, speed: int) -> None:
        """Drive both motors backward with the given duty cycle.

        Args:
            speed: PWM duty cycle in percent (0-100).
        """
        self.pwmA.ChangeDutyCycle(speed)
        self.pwmB.ChangeDutyCycle(speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

    def left(self, pwm: int) -> None:
        """Rotate left in place by driving wheels in opposite directions.

        Args:
            pwm: PWM duty cycle in percent (0-100).
        """
        self.pwmA.ChangeDutyCycle(pwm)
        self.pwmB.ChangeDutyCycle(pwm)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
    
    def right(self, pwm: int) -> None:
        """Rotate right in place by driving wheels in opposite directions.

        Args:
            pwm: PWM duty cycle in percent (0-100).
        """
        self.pwmA.ChangeDutyCycle(pwm)
        self.pwmB.ChangeDutyCycle(pwm)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

    def set_motor_speeds(self, left_speed: int, right_speed: int) -> None:
        """Set left and right motor speeds independently.

        Positive values drive forward, negative values drive backward.

        Args:
            left_speed: Left motor command in range -100 to 100.
            right_speed: Right motor command in range -100 to 100.
        """
        # Clamp speeds to -100 to 100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        if left_speed >= 0:
            # Forward on the left side
            self.pwmA.ChangeDutyCycle(left_speed)
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else:
            # Backward on the left side, abs since duty cycle must be positive
            self.pwmA.ChangeDutyCycle(abs(left_speed))
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)

        if right_speed >= 0:
            # Forward on the right side
            self.pwmB.ChangeDutyCycle(right_speed)
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        else:
            # Backward on the right side, abs since duty cycle must be positive
            self.pwmB.ChangeDutyCycle(abs(right_speed))
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)

    def left90(self) -> None:
        """Turn the robot left by approximately 90 degrees using gyro feedback."""
        if self.gyro is None:
            print("LEFT90 skipped: no gyro available")
            return

        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        yaw_angle = 0
        last_time = time.time()
        start_time = last_time
        while True:
            if abs(yaw_angle) >= self.TURN_90_ANGLE:
                self.stop()
                print(f"LEFT90 COMPLETE: {abs(yaw_angle):.1f}°")
                break

            print(f"TURN 90 LEFT: {abs(yaw_angle):.1f}°")
            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()

            if current_time - start_time > 20:  # safety timeout to prevent infinite loop
                print("LEFT90 aborted: timeout")
                self.stop()
                time.sleep(3)
                break

            dt = current_time - last_time
            last_time = current_time

            if abs(gyro_z) > self.GYRO_THRESHOLD:
                yaw_angle += gyro_z * dt


            
            # Small delay to stabilize loop rate and dt calculation
            time.sleep(0.01)

    def right90(self) -> None:
        """Turn the robot right by approximately 90 degrees using gyro feedback."""
        if self.gyro is None:
            print("RIGHT90 skipped: no gyro available")
            return

        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

        yaw_angle = 0
        last_time = time.time()
        start_time = last_time
        while True:
            if abs(yaw_angle) >= self.TURN_90_ANGLE:
                self.stop()
                print(f"RIGHT90 COMPLETE: {abs(yaw_angle):.1f}°")
                break

            print(f"TURN 90 RIGHT: {abs(yaw_angle):.1f}°")
            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()

            if current_time - start_time > 20:  # safety timeout to prevent infinite loop
                print("RIGHT90 aborted: timeout")
                self.stop()
                time.sleep(3)
                break

            dt = current_time - last_time
            last_time = current_time

            if abs(gyro_z) > self.GYRO_THRESHOLD:
                yaw_angle += gyro_z * dt

            
            # Small delay to stabilize loop rate and dt calculation
            time.sleep(0.01)


    def right45(self) -> None:
        """Turn the robot right by approximately 45 degrees using gyro feedback."""
        if self.gyro is None:
            print("RIGHT45 skipped: no gyro available")
            return

        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

        yaw_angle = 0
        last_time = time.time()
        start_time = last_time
        while True:
            if abs(yaw_angle) >= self.TURN_45_ANGLE:
                self.stop()
                print(f"RIGHT45 COMPLETE: {abs(yaw_angle):.1f}°")
                break

            print(f"TURN 90 RIGHT: {abs(yaw_angle):.1f}°")
            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()

            if current_time - start_time > 20:  # safety timeout to prevent infinite loop
                print("RIGHT45 aborted: timeout")
                self.stop()
                time.sleep(3)
                break

            dt = current_time - last_time
            last_time = current_time

            if abs(gyro_z) > self.GYRO_THRESHOLD:
                yaw_angle += gyro_z * dt
            
            # Small delay to stabilize loop rate and dt calculation
            time.sleep(0.01)

    def left45(self) -> None:
        """Turn the robot left by approximately 45 degrees using gyro feedback."""
        if self.gyro is None:
            print("LEFT45 skipped: no gyro available")
            return

        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        yaw_angle = 0
        last_time = time.time()
        start_time = last_time
        while True:
            if abs(yaw_angle) >= self.TURN_45_ANGLE:
                self.stop()
                print(f"LEFT45 COMPLETE: {abs(yaw_angle):.1f}°")
                break

            print(f"TURN 90 LEFT: {abs(yaw_angle):.1f}°")
            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()

            if current_time - start_time > 20:  # safety timeout to prevent infinite loop
                print("LEFT45 aborted: timeout")
                self.stop()
                time.sleep(3)
                break

            dt = current_time - last_time
            last_time = current_time

            if abs(gyro_z) > self.GYRO_THRESHOLD:
                yaw_angle += gyro_z * dt
   
            # Small delay to stabilize loop rate and dt calculation
            time.sleep(0.01)

    def turn_around180(self) -> None:
        """Turn the robot left by approximately 180 degrees using gyro feedback."""
        if self.gyro is None:
            print("TURN180 skipped: no gyro available")
            return

        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        yaw_angle = 0
        last_time = time.time()
        start_time = last_time

        while True:
            if abs(yaw_angle) >= self.TURN_180_ANGLE:
                self.stop()
                break

            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()

            if current_time - start_time > 20:  # safety timeout to prevent infinite loop
                print("TURN180 aborted: timeout")
                self.stop()
                time.sleep(3)
                break

            dt = current_time - last_time
            yaw_angle += gyro_z * dt
            last_time = current_time

    

if __name__ == "__main__":
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
    except:
        raise Exception("GPIO setup failed, could not initalize GPIO.BCM mode")
    
    gyro = gyroscope.MPU6050()
    motor = Motors(gyro)
    while True:
        motor.forward(50)