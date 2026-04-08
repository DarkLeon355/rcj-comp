import RPi.GPIO as GPIO
import time
import gyroscope
import numpy as np

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

class Motors:
    def __init__(self, gyro):
        self.gyro = gyro
        GPIO.setwarnings(False)


        # Motor pins
        self.IN1 = 24
        self.IN2 = 23
        self.ENA = 12  # PWM for motor 1
        self.IN3 = 17
        self.IN4 = 27
        self.ENB = 13  # PWM for motor 2

        # Setup GPIO
        for pin in [self.IN1, self.IN2, self.ENA, self.IN3, self.IN4, self.ENB]:
            GPIO.setup(pin, GPIO.OUT)

        # Setup PWM at 1kHz
        self.pwmA = GPIO.PWM(self.ENA, 1000)
        self.pwmB = GPIO.PWM(self.ENB, 1000)

        # Start PWM at 0% duty
        self.pwmA.start(0)
        self.pwmB.start(0)

        self.GYRO_THRESHOLD = 2  # degrees/sec, ignore small jitter
        self.turn_speed = 100

    def forward(self, speed):
        self.pwmA.ChangeDutyCycle(speed)
        self.pwmB.ChangeDutyCycle(speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

    def backward(self, speed):
        self.pwmA.ChangeDutyCycle(speed)
        self.pwmB.ChangeDutyCycle(speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

    def stop(self):
        self.pwmA.ChangeDutyCycle(0)
        self.pwmB.ChangeDutyCycle(0)

    def set_motor_speeds(self, left_speed, right_speed):
        """
        Sets speed for both motors.
        :param left_speed: Speed for left motor (-100 to 100)
        :param right_speed: Speed for right motor (-100 to 100)
        """
        # Clamp speeds to -100 to 100
        #left_speed = max(min(left_speed, 100), -100)
        #right_speed = max(min(right_speed, 100), -100)

        # Left Motor (pwmA)
        # Assuming typical driver logic:
        # Forward: A: speed, IN1: LOW, IN2: HIGH (Based on 'left' calling IN1=LOW, IN2=HIGH for forward turn?)
        # Wait, let's verify direction from `forward` vs `left` vs `right`.
        # `forward`: IN1=HIGH, IN2=LOW, IN3=HIGH, IN4=LOW.
        
        if left_speed >= 0:
            # Forward
            self.pwmA.ChangeDutyCycle(left_speed)
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else:
            # Backward
            self.pwmA.ChangeDutyCycle(abs(left_speed))
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)

        if right_speed >= 0:
            # Forward
            self.pwmB.ChangeDutyCycle(right_speed)
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        else:
            # Backward
            self.pwmB.ChangeDutyCycle(abs(right_speed))
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)

    def left(self, pwm):
        self.pwmA.ChangeDutyCycle(pwm)
        self.pwmB.ChangeDutyCycle(pwm)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
    
    def right(self, pwm):
        self.pwmA.ChangeDutyCycle(pwm)
        self.pwmB.ChangeDutyCycle(pwm)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)


    def left90(self):
        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        yaw_angle = 0
        last_time = time.time()
        while True:
                
            print(f"TURN 90 LEFT: {abs(yaw_angle):.1f}°")
            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()
            dt = current_time - last_time

            if abs(gyro_z) > self.GYRO_THRESHOLD:
                yaw_angle += gyro_z * dt
                last_time = current_time
            else:
                last_time = current_time

            if abs(yaw_angle) >= 85:
                self.stop()
                print(f"LEFT90 COMPLETE: {abs(yaw_angle):.1f}°")
                break
            
            # Small delay to stabilize loop rate and dt calculation
            time.sleep(0.01)

    def right90(self):
        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

        yaw_angle = 0
        last_time = time.time()
        while True:
                
            print(f"TURN 90 RIGHT: {abs(yaw_angle):.1f}°")
            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()
            dt = current_time - last_time

            if abs(gyro_z) > self.GYRO_THRESHOLD:
                yaw_angle += gyro_z * dt
                last_time = current_time
            else:
                last_time = current_time

            if abs(yaw_angle) >= 85:
                self.stop()
                print(f"RIGHT90 COMPLETE: {abs(yaw_angle):.1f}°")
                break
            
            # Small delay to stabilize loop rate and dt calculation
            time.sleep(0.01)


    def right45(self):
        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

        yaw_angle = 0
        last_time = time.time()
        while True:
                
            print(f"TURN 90 RIGHT: {abs(yaw_angle):.1f}°")
            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()
            dt = current_time - last_time

            if abs(gyro_z) > self.GYRO_THRESHOLD:
                yaw_angle += gyro_z * dt
                last_time = current_time
            else:
                last_time = current_time

            if abs(yaw_angle) >= 40:
                self.stop()
                print(f"RIGHT45 COMPLETE: {abs(yaw_angle):.1f}°")
                break
            
            # Small delay to stabilize loop rate and dt calculation
            time.sleep(0.01)

    def left45(self):
        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        yaw_angle = 0
        last_time = time.time()
        while True:
                
            print(f"TURN 90 LEFT: {abs(yaw_angle):.1f}°")
            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()
            dt = current_time - last_time

            if abs(gyro_z) > self.GYRO_THRESHOLD:
                yaw_angle += gyro_z * dt
                last_time = current_time
            else:
                last_time = current_time

            if abs(yaw_angle) >= 40:
                self.stop()
                print(f"LEFT45 COMPLETE: {abs(yaw_angle):.1f}°")
                break
            
            # Small delay to stabilize loop rate and dt calculation
            time.sleep(0.01)

    def turn_around180(self):
        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        yaw_angle = 0
        last_time = time.time()

        while True:
            gyro_z = self.gyro.get_gyro_z()
            current_time = time.time()
            dt = current_time - last_time
            yaw_angle += gyro_z * dt
            last_time = current_time

            if abs(yaw_angle) >= 170:
                self.stop()
                break
    

if __name__ == "__main__":
    gyro = gyroscope.MPU6050()
    motor = Motors(gyro)
    while True:
        motor.right(100)