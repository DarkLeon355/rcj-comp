import HC_SR04
import time
from typing import Any

class obstacle:
    def __init__(self, motors: Any) -> None:
        """Initialize obstacle sensors, motor controller, and avoidance parameters.

        Args:
            motors: Motor controller instance used for movement commands.
        """
        
        #─────────────────────────────────────────────────────────────
        # SETUP:
        #─────────────────────────────────────────────────────────────
        # sensor setup, refer the pins from the schematic
        try:
            self.vorne = HC_SR04.HC_SR04(4, 25)
            self.links = HC_SR04.HC_SR04(20, 21)
            self.rechts = HC_SR04.HC_SR04(19, 16)
        except:
            raise Exception("Error in obstacle avoidance, could not intialize HC_SR04 sensors")

        # motor setup
        try:
            self.motors = motors
        except:
            raise Exception("Error in obstacle avoidance, could not intialize motors")
        #─────────────────────────────────────────────────────────────

        #─────────────────────────────────────────────────────────────
        # DEFINITIONS:
        #─────────────────────────────────────────────────────────────
        # speed for searching an obstacle
        self.search_speed = 35
        #speed for driving past the obstacle
        self.drive_past_speed = 35
        # time to drive forward after passing the obstacle, this needs to be adjusted according to the robot's speed and size
        self.drive_past_obstacle_time = 1.0
        # distance threshold in cm for obstacle detection
        self.obstacle_threshold_cm = 10
        #─────────────────────────────────────────────────────────────

    def check_obstacle(self, sensor: Any) -> bool:
        """Check whether a sensor currently sees an obstacle within threshold.

        Args:
            sensor: Distance sensor object with a measure() method.

        Returns:
            True if obstacle is detected within obstacle_threshold_cm, else False.
        """
        distance = sensor.measure()
        if distance is not None and distance < self.obstacle_threshold_cm:
            return True
        else:
            return False

    def check_sides(self) -> None:
        """Update left/right obstacle availability flags from side sensors.
        
        Returns:
            Sets self.left and self.right to False if an obstacle is detected within obstacle_threshold_cm on the respective side, else True.
        """
        self.left = False
        self.right = False
        left_distance = self.links.measure()
        right_distance = self.rechts.measure()
        if (left_distance is None or left_distance > self.obstacle_threshold_cm):
            self.left = True
        if (right_distance is None or right_distance > self.obstacle_threshold_cm):
            self.right = True


    def search_for_obstacle_side_sensor(self, sensor: Any, thresh: float) -> None:
        """Move forward/backward until side sensor detects an obstacle.

        Args:
            sensor: Distance sensor object with a measure() method.
            thresh: Detection threshold in centimeters.
        """
        direction = 1
        counter = 0
        start_time = time.time()
        
        while True:
            distance = sensor.measure()
            # Check if obstacle found (handle None from sensor timeout)
            if distance is not None and distance < thresh:
                print(f"Found obstacle at {distance:.1f} cm")
                self.motors.stop()
                break
            elif direction == 1:
                if counter == 0:
                    print(f"Moving FORWARD (thresh={thresh:.0f})")
                self.motors.forward(self.search_speed)
                if counter > thresh:
                    direction = -1
                    counter = 0
                    thresh = thresh * 1.2

            else:
                if counter == 0:
                    print(f"Moving BACKWARD (thresh={thresh:.0f})")
                self.motors.backward(self.search_speed)
                if counter > thresh:
                    direction = 1
                    counter = 0
                    thresh = thresh * 1.2
            
            if time.time() - start_time > 30:  # Timeout after 30 seconds to prevent infinite loop
                self.motors.stop()
                raise Exception("Timeout while searching for obstacle with side sensor.")

            counter += 1
            time.sleep(0.01)
            

    def drive_past_obstacle(self, sensor: Any, lost_threshold: float, speed: int) -> float:
        """Drive forward until obstacle is no longer detected by a side sensor.

        Args:
            sensor: Distance sensor object with a measure() method.
            lost_threshold: Distance in centimeters that counts as obstacle lost.
            speed: Forward speed command.

        Returns:
            Elapsed time in seconds spent driving past the obstacle.
        """
     
        start_time = time.time()
        self.motors.forward(speed)
        
        while True:
            distance = sensor.measure()
            if distance is None or distance > lost_threshold:
                end_time = time.time()
                time_took = end_time - start_time
                self.motors.stop()
                print(f"Obstacle passed!")
                break

            if time.time() - start_time > 30:  # Timeout after 30 seconds to prevent infinite loop
                self.motors.stop()
                raise Exception("Timeout while trying to drive past obstacle.")
            
            time.sleep(0.01)  # Check every 10ms

        return time_took


    def drive_until_obstacle_found(self, speed: int, sensor: Any) -> None:
        """Drive forward until the given side sensor detects an obstacle.

        Args:
            speed: Forward speed command.
            sensor: Distance sensor object with a measure() method.
        """
        self.motors.forward(speed)
        start_time = time.time()
        while True:
            distance = sensor.measure()
            if distance is not None and distance < self.obstacle_threshold_cm:
                self.motors.stop()
                break
            time.sleep(0.05)
            if time.time() - start_time > 30:  # Timeout after 30 seconds to prevent infinite loop
                self.motors.stop()
                raise Exception("Timeout while trying to find obstacle.")


    def avoid_obstacle_right(self, speed: int) -> None:
        """Perform right-side obstacle avoidance maneuver.

        Args:
            speed: Cruise speed used for short forward segments.
        """
        # Initial turn to the right
        self.motors.right90()
        self.motors.stop()
        time.sleep(0.5)

        # Drive until obstacle not found on the left side anymore
        time_took = self.drive_past_obstacle(self.links, lost_threshold=20, speed=self.drive_past_speed)
        time.sleep(0.5)

        # Drive a bit forward for the next turn
        self.motors.forward(speed)
        time.sleep(self.drive_past_obstacle_time) # This needs to be adjusted according to the robot's speed and size
        self.motors.stop()
        time.sleep(0.5)

        #turn left to drive alongside the obstacle
        self.motors.left90()
        self.motors.stop()
        time.sleep(0.5)

        #drive a bit forward until the obstacle is detected again on the left side
        self.drive_until_obstacle_found(self.search_speed, self.links)
        time.sleep(0.5)

        # drive past the obstacle until it's no longer detected on the left side
        self.drive_past_obstacle(self.links, lost_threshold=20, speed=self.drive_past_speed)
        self.motors.stop()
        time.sleep(0.5)

        # again move forward a bit for the next turn
        self.motors.forward(speed)
        time.sleep(self.drive_past_obstacle_time) # This needs to be adjusted according to the robot's speed and size
        self.motors.stop()
        time.sleep(0.5)

        # turn left to get parallel to the obstacle 
        self.motors.left90()
        self.motors.stop()
        time.sleep(0.5)

        # drive forward until the obstacle is detected again on the left side
        self.drive_until_obstacle_found(self.search_speed, self.links)
        self.motors.stop()
        time.sleep(0.5)

        # now drive forward exactly as long as it took to for driving past the obstacle the first time, this should get us past the obstacle
        self.motors.forward(self.drive_past_speed)
        time.sleep(time_took)
        self.motors.stop()
        time.sleep(0.5)

        # now turn back to the original direction
        self.motors.right90()
        self.motors.stop()
        time.sleep(0.5)


    def avoid_obstacle_left(self, speed: int) -> None:
        """Perform left-side obstacle avoidance maneuver.

        Args:
            speed: Cruise speed used for short forward segments.
        """
        # Initial turn to the left
        self.motors.left90()
        self.motors.stop()
        time.sleep(0.5)

        # Drive until obstacle not found on the right side anymore
        time_took = self.drive_past_obstacle(self.rechts, lost_threshold=20, speed=self.drive_past_speed)
        time.sleep(0.5)

        # Drive a bit forward for the next turn
        self.motors.forward(speed)
        time.sleep(self.drive_past_obstacle_time) # This needs to be adjusted according to the robot's speed and size
        self.motors.stop()
        time.sleep(0.5)

        # Turn right to drive alongside the obstacle
        self.motors.right90()
        self.motors.stop()
        time.sleep(0.5)

        # Drive a bit forward until the obstacle is detected again on the right side
        self.drive_until_obstacle_found(self.search_speed, self.rechts)
        time.sleep(0.5)

        # Drive past the obstacle until it's no longer detected on the right side
        self.drive_past_obstacle(self.rechts, lost_threshold=20, speed=self.drive_past_speed)
        self.motors.stop()
        time.sleep(0.5)

        # Again move forward a bit for the next turn
        self.motors.forward(speed)
        time.sleep(self.drive_past_obstacle_time) # This needs to be adjusted according to the robot's speed and size
        self.motors.stop()
        time.sleep(0.5)

        # Turn right to get parallel to the obstacle
        self.motors.right90()
        self.motors.stop()
        time.sleep(0.5)

        # Drive forward until the obstacle is detected again on the right side
        self.drive_until_obstacle_found(self.search_speed, self.rechts)
        self.motors.stop()
        time.sleep(0.5)

        # Drive forward as long as the first pass took, this should get us past the obstacle
        self.motors.forward(self.drive_past_speed)
        time.sleep(time_took)
        self.motors.stop()
        time.sleep(0.5)

        # Turn back to the original direction
        self.motors.left90()
        self.motors.stop()
        time.sleep(0.5)


    def avoider(self) -> None:
        """
        Run obstacle avoidance decision logic based on front and side sensors. 
        Default to left side if both sides are free.
        Stop if an obstacle is detected in front but no free side is available.
        """
        if self.check_obstacle(self.vorne):
            self.check_sides()
            if self.left:
                self.avoid_obstacle_left(50)
            elif self.right:
                self.avoid_obstacle_right(50)
            else:
                print("Obstacle detected in front but no free side, stopping.")
                self.motors.stop()
    

if __name__ == "__main__":
    import motors
    import gyroscope
    import RPi.GPIO as GPIO

    try:
        GPIO.setmode(GPIO.BCM)
    except:
        print("GPIO setup failed, maybe not running on Raspberry Pi?")
        raise Exception("GPIO setup failed, could not initalize GPIO.BCM mode")

    gyro = gyroscope.MPU6050()
    motor = motors.Motors(gyro)
    o = obstacle(motor)
    o.avoider()

