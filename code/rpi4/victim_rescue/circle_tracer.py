import time
import cv2
import gyroscope
from find_circle import FindCircle
from motors import Motors


class CircleTracer:
    """
    Follows the biggest detected circle using the camera.
    Steers left/right to keep the circle horizontally centered in the frame.
    Stops as soon as the circle disappears.
    """

    

    def __init__(self):
        self.detector = FindCircle()
        gyro = gyroscope.MPU6050()
        self.motors = Motors(gyro)

        # ── Tuning ─────────────────────────────────────────────────────────────
        self.BASE_SPEED    = 50   # base forward speed (0-100)
        self.K_OFFSET      = 0.3  # gain: pixels off-centre → steering value
        self.ACTIVE_DEADZONE = 10 # |steering| below this → drive straight
        self.MAX_TURN_SPEED  = 40 # maximum speed contribution from steering
        self.MIN_TURN_SPEED  = 80 # cap for outer wheel speed
        self.LOOP_DELAY    = 0.1  # seconds the robot drives between each stop-capture cycle
        # ───────────────────────────────────────────────────────────────────────

    # ------------------------------------------------------------------
    def _steer(self, circle_x: int, frame_cx: int):
        """
        Differential steering toward the circle centre.

        steering > 0  → circle is to the RIGHT  → turn right
        steering < 0  → circle is to the LEFT   → turn left
        Within ACTIVE_DEADZONE                  → drive straight
        """
        offset   = circle_x - frame_cx                 # pixels off-centre
        steering = self.K_OFFSET * offset

        print(
            f"  circle_x={circle_x:4d}  offset={offset:+.0f}  steering={steering:.2f}"
        )

        if abs(steering) < self.ACTIVE_DEADZONE:
            self.motors.forward(self.BASE_SPEED)
        else:
            steer_mag    = min(abs(steering), self.MAX_TURN_SPEED)
            outer_speed  = int(min(self.BASE_SPEED + steer_mag, self.MIN_TURN_SPEED))
            inner_speed  = int(max(-self.BASE_SPEED - steer_mag, -self.MIN_TURN_SPEED))

            if steering > 0:
                # Turn right: left wheel outer, right wheel inner
                self.motors.set_motor_speeds(outer_speed, inner_speed)
            else:
                # Turn left: right wheel outer, left wheel inner
                self.motors.set_motor_speeds(inner_speed, outer_speed)

    # ------------------------------------------------------------------
    def trace(self):
        """
        Main tracing loop.
        Runs until the circle is no longer detected in a frame.
        """
        print("[CircleTracer] Starting – tracing biggest circle …")

        try:
            while True:
                # ── 1. Grab a fresh frame ──────────────────────────────
                self.motors.stop()
                time.sleep(0.5)
                self.detector.capture_img()

                # ── 2. Read frame width from the actual captured image ──
                frame_cx = self.detector.img.shape[1] // 2

                # ── 3. Detect the biggest circle ───────────────────────
                self.detector.find_biggest_circle()

                # ── 4. React ───────────────────────────────────────────
                if self.detector.circle:
                    print(
                        f"[TracerLoop] Circle at ({self.detector.x}, {self.detector.y})  "
                        f"r={self.detector.r}  frame_cx={frame_cx}"
                    )
                    self._steer(self.detector.x, frame_cx)
                else:
                    print("[CircleTracer] Circle lost – stopping.")
                    self.motors.stop()
                    break

                time.sleep(self.LOOP_DELAY)

        except KeyboardInterrupt:
            print("[CircleTracer] Interrupted by user.")

        finally:
            self.motors.stop()
            print("[CircleTracer] Motors stopped.")


# ── Standalone entry-point ──────────────────────────────────────────────────
if __name__ == "__main__":
    tracer = CircleTracer()
    tracer.trace()
