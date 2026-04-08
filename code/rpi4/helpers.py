import time
import numpy as np
import cv2
from typing import Any, List, Optional, Sequence, Tuple


class Helpers:
    """Utility helpers for obstacle handling, junction logic, and vision support."""

    def __init__(self, obstacle: Any, motor: Any) -> None:
        """Initialize helper parameters and runtime buffers.

        Args:
            obstacle: Obstacle handler instance used for front/side sensing logic.
            motor: Motor controller used for junction turn maneuvers.
        """
        #─────────────────────────────────────────────────────────────
        # SETUP:
        #─────────────────────────────────────────────────────────────
        self.obstacle = obstacle
        self.motor = motor
        #─────────────────────────────────────────────────────────────
        
        #─────────────────────────────────────────────────────────────
        # DEFINITIONS:
        #─────────────────────────────────────────────────────────────
        #Parameters for junction processing
        self.iterations_since_last_turn = 20
        self.min_iterations_btw_turns = 30
        self.center_tolerance_y = 90
        self.center_tolerance_x = 200

        #Parameters for obstacle approaching
        self.obstacle_approach_cm = 20     # distance at which robot starts slowing/centering
        self.obstacle_approach_speed = 30  # reduced speed when approaching obstacle

        #Parameters for x rotation smoothing
        self.x_rot_buffer = []       # rolling buffer for incline smoothing
        self.x_rot_buffer_size = 5  # average over last 10 readings

        #Parameters for y_level generation
        self.bottom_fraction=0.3
        self.bottom_margin=50
        self.y_levels_amount = 10

        #Parameters for dot merging
        self.max_dist = 300

        #Parameters for branch checking
        self.branch_window = 50  
        self.branch_min_win_pixels = 750
        #─────────────────────────────────────────────────────────────
        

    def junction_process_logic_helper(
        self,
        filtered_dots: Sequence[Tuple[int, int]],
        junction_center: Optional[Tuple[int, int]],
        cx_list: Sequence[int],
        cy_list: Sequence[int],
        frame_dimensions: Tuple[int, int],
    ) -> bool:
        """Handle junction turn decisions based on detected green markers.

        Args:
            filtered_dots: Merged green marker positions below the junction.
            junction_center: Junction center (x, y) or None.
            cx_list: X positions of detected line centroids.
            cy_list: Y positions of detected line centroids.
            frame_dimensions: Frame dimensions as (height, width).
        """

        height = frame_dimensions[0]
        width = frame_dimensions[1]

        self.iterations_since_last_turn += 1
        skip_frame_flag = False

        if len(filtered_dots) == 0:
            return False

        if self.iterations_since_last_turn > self.min_iterations_btw_turns:

            if (junction_center is not None
                    and abs(junction_center[1] - height // 2) < self.center_tolerance_y
                    and abs(junction_center[0] - width // 2) < self.center_tolerance_x):
        
                if len(filtered_dots) == 2 and junction_center is not None:
                    skip_frame_flag = True
                    self.motor.turn_around180()
                    self.motor.forward(50)  # Move forward a bit after turning
                    time.sleep(0.25)
                    self.iterations_since_last_turn = 0    

                if len(filtered_dots) == 1 and cx_list and cy_list and junction_center is not None:
                    self.motor.stop()
                    print("Single green dot detected, checking position relative to line...")
                    cx_green, cy_green = filtered_dots[0]

                    # Find centroid whose y is closest to green-dot y
                    closest_idx = int(np.argmin(np.abs(np.array(cy_list) - cy_green)))
                    cx_at_green = cx_list[closest_idx]
                    skip_frame_flag = True

                    if cx_green < cx_at_green:
                        self.motor.left90()
                        self.motor.forward(50)  # Move forward a bit after turning
                        time.sleep(0.25)
                        self.iterations_since_last_turn = 0
                       
                    elif cx_green > cx_at_green:
                        self.motor.right90()
                        self.motor.forward(50)  # Move forward a bit after turning
                        time.sleep(0.25)
                        self.iterations_since_last_turn = 0
                      

                return skip_frame_flag

            else: 
                print("Junction center not within tolerance, skipping junction processing.")
                return False

        return False
        
    def dot_merger_helper(self, points: Sequence[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Merge nearby points and return averaged centers.

        Args:
            points: Iterable of (x, y) points.

        Returns:
            List of merged (x, y) centers.
        """
        if not points:
            return []
        pts = [tuple(map(int, p)) for p in points]
        used = [False] * len(pts)
        merged = []
        max_d2 = self.max_dist * self.max_dist
        for i, (xi, yi) in enumerate(pts):
            if used[i]:
                continue
            cx, cy, cnt = xi, yi, 1
            used[i] = True
            for j in range(i + 1, len(pts)):
                if used[j]:
                    continue
                xj, yj = pts[j]
                if (xi - xj) * (xi - xj) + (yi - yj) * (yi - yj) <= max_d2:
                    cx += xj
                    cy += yj
                    cnt += 1
                    used[j] = True
            merged.append((int(round(cx / cnt)), int(round(cy / cnt))))
        return merged
    

    def check_branch_helper(self, x0: int, y0: int, threshold: np.ndarray) -> bool:
        """Check whether a branch candidate window has enough line pixels.

        Args:
            x0: Probe x-position.
            y0: Probe y-position.

        Returns:
            True when the local ROI contains at least branch_min_win_pixels.
        """
        win = self.branch_window
        x1 = max(x0 - win, 0)
        x2 = min(x0 + win, threshold.shape[1])
        y1 = max(y0 - win, 0)
        y2 = min(y0 + win, threshold.shape[0])
        if x1 >= x2 or y1 >= y2:
            return False
        roi = threshold[y1:y2, x1:x2]
        return cv2.countNonZero(roi) >= self.branch_min_win_pixels
    

    def obstacle_approaching_helper(self) -> bool:
        """Update obstacle-approaching state and run avoidance when necessary."""
        front_distance = self.obstacle.vorne.measure()

        if front_distance is not None and front_distance < self.obstacle.obstacle_threshold_cm:
            # obstacle close, start handler
            self.obstacle.avoider()
            obstacle_approaching = False
        elif front_distance is not None and front_distance < self.obstacle_approach_cm:
            # obstacle approaching, start slowing and centering
            obstacle_approaching = True
        else:
            # no obstacle ahead
            obstacle_approaching = False

        return obstacle_approaching

    def rotation_helper(self, x_rot_unsmoothed: float) -> float:
        """Return smoothed x-rotation using a rolling average buffer.

        Args:
            x_rot_unsmoothed: Raw x-rotation angle.

        Returns:
            Smoothed x-rotation angle.
        """
        self.x_rot_buffer.append(x_rot_unsmoothed)
        if len(self.x_rot_buffer) > self.x_rot_buffer_size:
            self.x_rot_buffer.pop(0)
        return sum(self.x_rot_buffer) / len(self.x_rot_buffer)
    
    def generate_y_levels_helper(self, height: int) -> np.ndarray:
        """Generate scanline y-levels with slightly higher density near the bottom.

        Args:
            height: Frame height in pixels.

        Returns:
            NumPy array of y-level indices.
        """
        start = max(height - 1 - self.bottom_margin, 0)
        end = int(height * self.bottom_fraction)

        t = np.linspace(1, 0.0, self.y_levels_amount)  # start from bottom (t=1) to top (t=0)
        bias_power = 1  # <1 → more bottom emphasis, close to 1 keeps it gentle
        weights = np.power(t, bias_power)

        y_float = end + weights * (start - end)
        y_indices = np.clip(np.rint(y_float).astype(int), 0, height - 1)

        # keep order while removing duplicates caused by rounding
        unique_levels = []
        last = None
        for val in y_indices:
            if last is None or val != last:
                unique_levels.append(val)
                last = val

        y_levels = np.array(unique_levels, dtype=int)
        return y_levels
