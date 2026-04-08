import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2
import subprocess
from typing import List, Optional, Sequence, Tuple

import save_img
import motors
import gyroscope 
import obstacle
import helpers


class LineFollow:
    """Main line-following pipeline with junction and obstacle handling."""

    def __init__(self) -> None:
        """Initialize parameters, camera, and hardware helper modules."""

        #─────────────────────────────────────────────────────────────
        # DEFINITIONS:
        #─────────────────────────────────────────────────────────────
        # Flags and state variables
        self.junction_flag = False
        self.obstacle_approaching_flag = False

        # frame initialization
        self.frame = None
        self.output_frame = None 

        #list initialization
        self.cx_list = []
        self.cy_list = []  
        self.green_dots = []
        self.large_contours = []

        #variable initialization
        self.angle_deg = 0.0
        self.neigung = 0
        self.framecount = 0
        self.junction_bbox = None # Bounding box einer Kreuzung (x, y, w, h) wenn vorhanden
        self.junction_center = None


        #─────────────────────────────────────────────────────────────
        # DEBUGGING: test
        #─────────────────────────────────────────────────────────────
        # debug options, be aware the picture can be very ugly if you enable too many
        self.debug_draw_raw_green = False  # draw unmerged green contours
        self.debug_draw_junction_center = True # draws the junction center if found
        self.debug_draw_filtered_green_dots = True # draw merged green dots
        self.debug_draw_line_centroids = True # draw line centroids
        self.debug_draw_areas = False  # print areas of detected contour slices
        self.debug_draw_centroid_offsets = True # draw distance from centroids to frame center
        self.debug_draw_connected_centroids = True  # draw lines connecting centroids
        self.save_pics = True # save pics locally
        self.stream_to_pc = True # stream pics via socket


        #─────────────────────────────────────────────────────────────
        # PARAMETERS:
        #─────────────────────────────────────────────────────────────
        # green dot parameters:
        self.MIN_GREEN_DOT_AREA = 100 #minimale fläche die ein grüner punkt haben muss (scaled for 1280x960)
        # green ranges(HSV): 
        # Range A: gelb-grün
        self.lower_green_a = np.array([20, 40, 20])
        self.upper_green_a = np.array([50, 255, 255])
        # Range B: grün-türkis
        self.lower_green_b = np.array([50, 30, 20])
        self.upper_green_b = np.array([110, 255, 255])

        # parameters for line detection:
        self.y_levels_amount = 10
        self.MIN_LINE_AREA = 500 
        self.MAX_LINE_AREA = 2500 

        # junction parameters:
        self.MIN_JUNCTION_AREA = 20000 #minimale area der junction -> area wird am output bild printed (scaled for 1280x960)
        self.MAX_JUNCTION_AREA = 60000 #maximale area der junction

        # junction check distances
        self.junction_validation_up = 175
        self.junction_validation_down = 175
        self.junction_validation_left = 250
        self.junction_validation_right = 250

        # Junction kernels:
        # Needs to be longer than line width and height to detect junction arms properly
        self.long_kernel_parameter_junction = 400 # this is the parameter for horizontal and vertical kernel, for junction detection

        #Motor speeds (dont go below 30)
        self.base_speed_original = 40  # Standartgeschwindigkeit -> Achtung zu hohe Geschwindkeit kann Fehler im Linefollowing verursachen, may get changed due to incline or decline
        self.junction_speed = 35 # Speed detecting a junction 
        self.max_speed = 100   # Der maximale Speed
        self.min_turn_speed = 50 # Die Geschwindgkeit mit der drehungen mindestens gefahren werden, should be at least 60
        self.max_turn_speed = 100 # adjust in case of overturning
        self.obstacle_approach_speed = 30 # reduced speed when approaching obstacle
        self.decline_speed = 30 # speed when going down an incline
        self.incline_speed = 100 # speed when going up an incline
        self.inner_reduction_ratio = 0.8 # how much the inner wheels are slower

        #cropping
        self.top = 0   # Significant top crop (remove far-ahead view)
        self.bottom = 300   # Keep more of the immediate front view (scaled to new resolution)
        self.left = 250     # Significant side cropping (scaled to new resolution)
        self.right = 250    # Significant side cropping (scaled to new resolution)

        #steering parameters
        self.K_angle = 3 # Einfluss des Winkels auf die Motoransteuerung
        self.K_offset = 0.3 # Einfluss des offset auf die Motoransteuerung
        self.deadzone_base = 20 # Deadzone for steering control, also Forward() solange unter self.deadzone
        self.min_points = 5 # minimum amout of points, if below it goes forward

        #─────────────────────────────────────────────────────────────


        #─────────────────────────────────────────────────────────────
        # Picamera2 setup
        #─────────────────────────────────────────────────────────────
        try:
            self.picam2 = Picamera2() 
            self.camera_config = self.picam2.create_video_configuration(main={"size": (1280, 960)}) # 4:3 aspect ratio
            self.picam2.configure(self.camera_config)
            self.picam2.start()
            time.sleep(2)
        except:
            raise Exception("Failed to initialize Picamera2, check camera connection and settings")
        #─────────────────────────────────────────────────────────────

        #─────────────────────────────────────────────────────────────
        # initialize selfmade libraries
        #─────────────────────────────────────────────────────────────
        try:
            # Stream to PC
            self.saver = save_img.save_img_and_stream(stream = self.stream_to_pc, local=True)
            self.sensor = gyroscope.MPU6050()
            self.motor = motors.Motors(self.sensor)
            self.obstacle = obstacle.obstacle(self.motor)
            self.helpers = helpers.Helpers(self.obstacle, self.motor)
       
        except:
            raise Exception("Error initializing custom libraries")
        
        #─────────────────────────────────────────────────────────────


    
    def get_green_mask(self) -> None:
        """Create and store the green mask for marker detection."""
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask_a = cv2.inRange(hsv, self.lower_green_a, self.upper_green_a)
        mask_b = cv2.inRange(hsv, self.lower_green_b, self.upper_green_b)
        green_mask = cv2.bitwise_or(mask_a, mask_b)
        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        self.green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
            
    def find_green_dots(self) -> None:
        """Detect green marker centroids and store filtered marker positions."""
        green_contours, _ = cv2.findContours(self.green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.green_dots.clear()
        for cnt in green_contours:
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Draw raw green markers only in debug mode
            if self.debug_draw_raw_green:
                cv2.circle(self.output_frame, (cx, cy), 6, (255, 0, 255), -1)  # magenta

            # Keep only sufficiently large dots for decision logic
            if cv2.contourArea(cnt) > self.MIN_GREEN_DOT_AREA:
                self.green_dots.append((cx, cy))

        # Optional overlay (keeps showing filtered count)
        cv2.putText(self.output_frame, f"green:{len(self.green_dots)}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2, cv2.LINE_AA)
        
    def get_threshold(self) -> None:
        """Create binary line mask from current frame and suppress green regions."""
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Try Otsu first
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # If Otsu fails (too bright background), fallback to manual
        white_ratio = np.sum(thresh == 255) / thresh.size
        if white_ratio > 0.4:
            _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=3)

        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=3)

        thresh[self.green_mask > 0] = 0

        self.threshold = thresh

    def find_junction(self) -> None:
        """Detect junction candidates and compute junction state for current frame."""
        #default at start
        junction_center = None
        junction_y = None
        self.junction_flag = False
        self.junction_bbox = None
        self.filtered_dots = []
        valid = []
        
        # Performance optimization: downsample before expensive morphological operations
        # Scale down by 3×
        scale_factor = 3
        small_h = self.threshold.shape[0] // scale_factor
        small_w = self.threshold.shape[1] // scale_factor
        threshold_small = cv2.resize(self.threshold, (small_w, small_h), interpolation=cv2.INTER_NEAREST)
        
        # Use proportionally smaller kernels on the downsampled image
        kernel_size = self.long_kernel_parameter_junction // scale_factor  # kernel also gets downsized
        horizontal_kernel_small = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, 1))
        vertical_kernel_small = cv2.getStructuringElement(cv2.MORPH_RECT, (1, kernel_size))
        
        # Build joint image from morphological opening using the smaller kernels on smaller image
        horizontal = cv2.morphologyEx(threshold_small, cv2.MORPH_OPEN, horizontal_kernel_small, iterations=1)
        vertical = cv2.morphologyEx(threshold_small, cv2.MORPH_OPEN, vertical_kernel_small, iterations=1)
        joints_small = cv2.bitwise_and(horizontal, vertical)
        
        # Scale back up to original size
        joints = cv2.resize(joints_small, (self.threshold.shape[1], self.threshold.shape[0]), interpolation=cv2.INTER_NEAREST)
        
        cnts, _ = cv2.findContours(joints, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if cnts:
            for cnt in cnts:
                area = cv2.contourArea(cnt)
                if self.MIN_JUNCTION_AREA <= area <= self.MAX_JUNCTION_AREA:
                    valid.append((area, cnt))

            if valid:
                area, largest_joint = max(valid, key=lambda x: x[0])
                M = cv2.moments(largest_joint)
                if M["m00"] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    junction_center = (cx, cy)

                    # Check points should be beyond the line width to detect junction arms
                    check_up = cy - self.junction_validation_up
                    check_left = cx - self.junction_validation_left
                    check_right = cx + self.junction_validation_right
                    check_down = cy + self.junction_validation_down

                    number_of_branches = 0

                    if check_up > 0 and self.helpers.check_branch_helper(cx, check_up, self.threshold):
                        number_of_branches += 1
                        cv2.circle(self.output_frame, (cx, check_up), 8, (0, 0, 255), 2)
                    if check_left > 0 and self.helpers.check_branch_helper(check_left, cy, self.threshold):
                        number_of_branches += 1
                        cv2.circle(self.output_frame, (check_left, cy), 8, (0, 0, 255), 2)
                    if check_right < self.threshold.shape[1] and self.helpers.check_branch_helper(check_right, cy, self.threshold):
                        number_of_branches += 1
                        cv2.circle(self.output_frame, (check_right, cy), 8, (0, 0, 255), 2)
                    if check_down < self.threshold.shape[0] and self.helpers.check_branch_helper(cx, check_down, self.threshold):
                        number_of_branches += 1
                        cv2.circle(self.output_frame, (cx, check_down), 8, (0, 0, 255), 2)

                    if number_of_branches >= 3:
                        # Confirmed junction
                        self.junction_flag = True
                        self.junction_bbox = cv2.boundingRect(largest_joint)

                        x, y, w_box, h_box = self.junction_bbox

                        if self.debug_draw_junction_center:
                            cv2.drawContours(self.output_frame, [largest_joint], -1, (0, 255, 0), 3)
                            cv2.circle(self.output_frame, junction_center, 15, (36, 255, 12), -1)
                            cv2.putText(self.output_frame, f"A={int(area)}",
                                        (x, max(15, y - 5)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

                    else:
                        # Not a valid junction; leave cleared
                        print(f"Only {number_of_branches} branches")
                        junction_center = None
                        self.junction_flag = False
                        self.junction_bbox = None

        # If we have a junction center, collect green dots below it (merged)
        if junction_center:
            junction_y = junction_center[1]
            if self.green_dots:
                dots_below = [dot for dot in self.green_dots if dot[1] > junction_y] 
                self.filtered_dots = self.helpers.dot_merger_helper(dots_below)
                if self.debug_draw_filtered_green_dots:
                    for dot in self.filtered_dots:
                        cv2.circle(self.output_frame, dot, 5, (255, 0, 0), -1)
            else:
                self.filtered_dots = []

        # Store final state
        self.junction_center = junction_center #this is a tuple (x,y) or None

    def find_line_centroids(self) -> None:
        """Find line centroids across scanlines and update centroid lists."""
        self.cx_list.clear()
        self.cy_list.clear()
        self.large_contours.clear()
        prev = None
        max_dist_btw_cx = self.cropped_width/3

        for y in self.y_levels:
            cx = None
            cy = None
            y_start = max(0, y - 2)
            y_end = min(self.threshold.shape[0], y + 3)
            slice_line = self.threshold[y_start:y_end, :]
            contours, _ = cv2.findContours(slice_line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                self.large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.MIN_LINE_AREA and cv2.contourArea(cnt) < self.MAX_LINE_AREA]
                if self.large_contours:
                    # Draw all large contours for this scanline (yellow), shift by y_start
                    cv2.drawContours(self.output_frame, self.large_contours, -1, (0, 255, 255), 1, offset=(0, y_start))
                    for cnt in self.large_contours:
                        M = cv2.moments(cnt)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"]) + y_start
                            if prev is not None and abs(cx - prev) > max_dist_btw_cx:
                                cx = prev
                            # Keep tracking from the last accepted centroid to avoid stale comparisons.
                            prev = cx
                        

                            if self.debug_draw_areas and cx is not None and cy is not None:
                                area = cv2.contourArea(cnt)
                                cv2.putText(
                                    self.output_frame,
                                    f"{int(area)}",
                                    (cx + 50, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    (0, 0, 255),
                                    2,
                                    cv2.LINE_AA
                                )
                               

                    # Override cx if this y is within detected junction vertical span
                    if self.junction_center is not None and self.junction_bbox is not None and cx is not None:
                        jx, jy, jw, jh = self.junction_bbox
                        # small tolerance to include edge rows
                        if (jy - 10) <= y <= (jy + jh + 10): 
                            cx = self.junction_center[0]
                            # draw in different color to indicate forced alignment (orange)
                            if self.debug_draw_line_centroids:
                                cv2.circle(self.output_frame, (cx, int(y)), 6, (0, 140, 255), -1)
                        else:
                            if self.debug_draw_line_centroids:
                                cv2.circle(self.output_frame, (cx, int(y)), 5, (0, 255, 0), -1)
                    else:
                        if self.debug_draw_line_centroids and cx is not None:
                            cv2.circle(self.output_frame, (cx, int(y)), 5, (0, 255, 0), -1)

                    if cx is not None and cy is not None:
                        self.cx_list.append(cx)
                        self.cy_list.append(cy)


        if len(self.cx_list) > self.min_points:
            if self.debug_draw_centroid_offsets:
                self.draw_line_centroid_distances(self.cx_list, self.cy_list)
            self.fit_line_and_angle(self.cx_list, self.cy_list)


    def fit_line_and_angle(self, cx_list: Sequence[int], cy_list: Sequence[int]) -> None:
        """Fit a direction from centroids and update steering angle estimate.

        Args:
            cx_list: X positions of line centroids.
            cy_list: Y positions of line centroids.
        """
        x = 0
        dx_sum = 0
        dy_sum = 0

        n = min(len(cx_list), len(cy_list))
        if n >= 2:
            # iterate to the second-last index (avoid x+1 overflow)
            while x < n - 1:
                lower_point = cx_list[x]
                upper_point = cx_list[x+1]
                lower_y = cy_list[x]
                upper_y = cy_list[x+1]

                # connect adjacent centroids
                if self.debug_draw_connected_centroids:
                    cv2.line(self.output_frame, (int(lower_point), int(lower_y)),
                            (int(upper_point), int(upper_y)), (255, 0, 0), 2)

                delta_y = upper_y - lower_y
                delta_x = upper_point - lower_point
                x = x + 1

                dx_sum += delta_x
                dy_sum += delta_y

            angle_rad = np.arctan2(dx_sum, -dy_sum)
            self.angle_deg = angle_rad * 180.0 / np.pi

    def draw_steering_info(self, angle: float, offset: float, steering: float) -> None:
        """Draw angle, offset, and steering information on the output frame."""
        y_offset = 50  # Start position for text
        line_spacing = 30
        
        # Angle
        cv2.putText(self.output_frame, f"Angle: {angle:.1f}deg", 
                    (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)  # Black outline
        cv2.putText(self.output_frame, f"Angle: {angle:.1f}deg", 
                    (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)  # Yellow text
        
        # Offset
        cv2.putText(self.output_frame, f"Offset: {offset:+.0f}px", 
                    (10, y_offset + line_spacing), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(self.output_frame, f"Offset: {offset:+.0f}px", 
                    (10, y_offset + line_spacing), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        
        # Steering
        cv2.putText(self.output_frame, f"Steering: {steering:+.1f}", 
                    (10, y_offset + 2*line_spacing), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(self.output_frame, f"Steering: {steering:+.1f}", 
                    (10, y_offset + 2*line_spacing), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

    def draw_line_centroid_distances(self, cx_list: Sequence[int], cy_list: Sequence[int]) -> None:
        """Overlay centroid-to-center distances for debugging.

        Args:
            cx_list: X positions of detected centroids.
            cy_list: Y positions of detected centroids.
        """
        frame_center_x = self.cropped_width // 2
        number = 0

        for cx, y in zip(cx_list, cy_list):
            number += 1
            distance = cx - frame_center_x
            text = f"{number}: {distance:+d}"

            # Clamp text position to stay inside the frame
            text_x = min(max(cx + 10, 0), self.cropped_width - 40)
            text_y = min(max(y - 10, 20), self.output_frame.shape[0] - 10)
            text_pos = (text_x, text_y)

            # Draw black outline for visibility
            cv2.putText(
                self.output_frame,
                text,
                text_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 0),
                4,
                cv2.LINE_AA
            )
            # Draw yellow text
            cv2.putText(
                self.output_frame,
                text,
                text_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

    def calc_steering(self) -> None:
        """Compute wheel commands from line angle and lateral offset."""
        
        base_speed = self.base_speed
        # Detect if we're on an incline
        on_incline = self.neigung >= 10

        on_decline = self.neigung <= -10
        
        # Reduce steering sensitivity on inclines to prioritize forward motion
        if on_incline:
            print("On incline")
            # Cut steering gains significantly when climbing/descending
            base_speed = self.incline_speed
            K_angle = self.K_angle * 0.1  # Reduce angle influence by 90%
            K_offset = self.K_offset * 0.3  # Reduce offset influence by 70%
            active_deadzone = self.deadzone * 2  # Widen deadzone to go straight more often

        elif on_decline:
            print("On decline")
            base_speed = self.decline_speed
            K_angle = self.K_angle * 0.1  # Reduce angle influence by 90%
            K_offset = self.K_offset * 0.3  # Reduce offset influence by 70%
            active_deadzone = self.deadzone * 2  # Widen deadzone to go straight more often

        else:
            K_angle = self.K_angle
            K_offset = self.K_offset
            active_deadzone = self.deadzone


        if len(self.cx_list) <= self.min_points:
            print("No line detected, continuing forward")
            self.motor.forward(base_speed)
            return

        if self.obstacle_approaching_flag:
            base_speed = self.obstacle_approach_speed
            K_angle = self.K_angle * 0.5  # Reduce angle influence by 50%
            K_offset = self.K_offset  
            active_deadzone = self.deadzone * 2  # Widen deadzone to go straight more often
            print("Approaching obstacle")


        if self.cx_list:
        
            frame_center_x = self.cropped_width // 2
            avg_offset = np.median(self.cx_list) - frame_center_x

            steering = (K_angle * self.angle_deg) + K_offset * avg_offset
            
            print(f"{self.framecount}: follow: angle={self.angle_deg:.2f}°, offset={avg_offset:+.0f}, steering={steering:.2f}")
            
            # Draw steering info on output frame
            self.draw_steering_info(self.angle_deg, avg_offset, steering)

            if self.junction_flag:
                base_speed = self.junction_speed

            if abs(steering) < active_deadzone:  
                self.motor.forward(base_speed)
            else:
                # Differential steering: keep both wheels forward for smooth arc turns.
                steer_mag = abs(steering)
                outer_speed = int(min(max(base_speed + steer_mag, self.min_turn_speed), self.max_turn_speed))
                min_inner_speed = max(20, int(self.min_turn_speed * 0.5))
                inner_target = base_speed - steer_mag * self.inner_reduction_ratio
                inner_speed = int(min(max(inner_target, min_inner_speed), self.max_turn_speed))


                if steering > 0:
                    # Turn right: slow right wheel
                    self.motor.set_motor_speeds(outer_speed, inner_speed)
                else:
                    # Turn left: slow left wheel
                    self.motor.set_motor_speeds(inner_speed, outer_speed)



    def process_frame(self) -> None:
        """Process one camera frame and run the complete pipeline."""

        self.obstacle_approaching_flag = self.helpers.obstacle_approaching_helper()

        self.framecount += 1

        # Take frame and copy it for processing
        self.uncropped_frame = self.picam2.capture_array()
        self.uncropped_height, self.uncropped_width = self.uncropped_frame.shape[:2]  # get original (uncropped) dimensions

        self.frame = self.uncropped_frame[self.top:self.uncropped_height - self.bottom, self.left:self.uncropped_width - self.right].copy() # crop the image and make a copy
        self.cropped_height, self.cropped_width = (self.uncropped_height - self.top - self.bottom), (self.uncropped_width - self.left - self.right)  # calculate the cropped image size

        self.original_frame = self.frame.copy()
        self.output_frame = self.frame.copy()

        self.x_rot_unsmoothed = self.sensor.get_x_rotation()
        self.neigung = self.helpers.rotation_helper(self.x_rot_unsmoothed)
        
        self.base_speed = self.base_speed_original #always default to original base speed at start
        self.deadzone = self.deadzone_base #always default to original deadzone at start

        # First get the green mask
        self.get_green_mask()
        # Second, find the green dots if any
        self.find_green_dots()
        # Third, get the threshold for line detection
        self.get_threshold()
        # Fourth, find the junction if any
        self.find_junction()
        # Fifth, get the y-levels for line detection
        self.y_levels = self.helpers.generate_y_levels_helper(self.cropped_height)
        # Sixth, find the line centroids
        self.find_line_centroids()
        # Process logic based on the detected line and junction
        skip_flag = self.helpers.junction_process_logic_helper(
            self.filtered_dots,
            self.junction_center,
            self.cx_list,
            self.cy_list,
            (self.cropped_height, self.cropped_width),
        )
        # If skip_flag is set, skip the rest of the processing
        if skip_flag:
            skip_flag = False
            if self.save_pics or self.stream_to_pc:
                self.saver.save(
                    edited=self.output_frame,
                    original=self.original_frame,
                    binary=self.threshold,
                    green=self.green_mask,
                    cx_list=self.cx_list.copy(),
                    cy_list=self.cy_list.copy(),
                    junctionflag=self.junction_flag,
                    junctioncenter=self.junction_center,
                    greendots=self.green_dots.copy(),
                    count=self.framecount,
                )
            return
        # Calculate steering based on the centroids
        self.calc_steering()
        # Save the frames
        if self.save_pics or self.stream_to_pc:
            self.saver.save(
                edited=self.output_frame,
                original=self.original_frame,
                binary=self.threshold,
                green=self.green_mask,
                cx_list=self.cx_list.copy(),
                cy_list=self.cy_list.copy(),
                junctionflag=self.junction_flag,
                junctioncenter=self.junction_center,
                greendots=self.green_dots.copy(),
                count=self.framecount,
            )
        # end single iteration
 
if __name__ == "__main__":
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
    except:
        raise Exception("GPIO setup failed, could not initalize GPIO.BCM mode")

    try:
        subprocess.run(["bash", "delete.sh"])
    except:
        raise Exception("Failed to run delete old images")

    line_follower = LineFollow()
    try:
        while True:
            line_follower.process_frame()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        GPIO.cleanup()
        line_follower.motor.stop()
        line_follower.saver.close_file()
        time.sleep(1)
        line_follower.motor.stop()