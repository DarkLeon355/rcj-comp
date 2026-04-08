import cv2
from cv2 import VideoCapture
import numpy as np


def _noop(_value: int) -> None:
    """Trackbar callback that intentionally does nothing."""


cam = VideoCapture(1)
if not cam.isOpened():
    raise RuntimeError('Unable to access camera')

cv2.namedWindow('Detected Circle')
cv2.namedWindow('Circle Params', cv2.WINDOW_NORMAL)

# Trackbars make tuning parameters much quicker on the live feed
cv2.createTrackbar('dp x10', 'Circle Params', 10, 30, _noop)
cv2.createTrackbar('minDist', 'Circle Params', 100, 400, _noop)
cv2.createTrackbar('param1', 'Circle Params', 100, 300, _noop)
cv2.createTrackbar('param2', 'Circle Params', 40, 150, _noop)
cv2.createTrackbar('minRadius', 'Circle Params', 30, 200, _noop)
cv2.createTrackbar('maxRadius', 'Circle Params', 60, 250, _noop)
cv2.createTrackbar('blur k', 'Circle Params', 5, 15, _noop)
cv2.createTrackbar('canny low', 'Circle Params', 50, 255, _noop)
cv2.createTrackbar('canny high', 'Circle Params', 150, 255, _noop)

try:
    while True:
        ret, img = cam.read()
        if not ret:
            print('Frame grab failed, exiting loop')
            break

        blur_k = cv2.getTrackbarPos('blur k', 'Circle Params')
        blur_k = blur_k + 1 if blur_k % 2 == 0 else blur_k  # kernel must be odd
        blur_k = max(3, blur_k)

        output = img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, blur_k)

        low = cv2.getTrackbarPos('canny low', 'Circle Params')
        high = max(low + 1, cv2.getTrackbarPos('canny high', 'Circle Params'))
        edges = cv2.Canny(gray, low, high)
        gray_for_hough = cv2.GaussianBlur(edges, (blur_k, blur_k), 0)

        dp = max(1, cv2.getTrackbarPos('dp x10', 'Circle Params')) / 10.0
        min_dist = max(10, cv2.getTrackbarPos('minDist', 'Circle Params'))
        param1 = max(1, cv2.getTrackbarPos('param1', 'Circle Params'))
        param2 = max(1, cv2.getTrackbarPos('param2', 'Circle Params'))
        min_radius = cv2.getTrackbarPos('minRadius', 'Circle Params')
        max_radius = cv2.getTrackbarPos('maxRadius', 'Circle Params')
        if max_radius <= min_radius:
            max_radius = min_radius + 5

        circles = cv2.HoughCircles(
            gray_for_hough,
            cv2.HOUGH_GRADIENT,
            dp=dp,
            minDist=min_dist,
            param1=param1,
            param2=param2,
            minRadius=min_radius,
            maxRadius=max_radius,
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            x, y, r = circles[0][0]
            cv2.circle(output, (x, y), r, (0, 255, 0), 2)
            cv2.circle(output, (x, y), 2, (0, 0, 255), 3)
            cv2.putText(
                output,
                f'r={r}px',
                (x - 40, y - r - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                output,
                'No circles detected',
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow('Detected Circle', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
finally:
    cam.release()
    cv2.destroyAllWindows()