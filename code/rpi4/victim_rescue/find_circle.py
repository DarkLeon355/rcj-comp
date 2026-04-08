import cv2
import numpy as np

class FindCircle:
    def __init__(self):
        self.cam = cv2.VideoCapture(0)

    
    def capture_img(self):
        ret, self.img = self.cam.read()
        if not ret:
            return

    def find_biggest_circle(self):
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        dp = 1
        min_dist = 20
        param1 = 100 # High threshold for canny
        param2 = 30  # Accumulator threshold (Lower = more circles, higher = fewer/better)
        min_radius = 10
        max_radius = 300  
    
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=dp,
            minDist=min_dist,
            param1=param1,
            param2=param2,
            minRadius=min_radius,
            maxRadius=max_radius)

        if circles is not None:
            self.circle = True
            circles = np.round(circles[0]).astype(int)
            print(f"Found {len(circles)} circles total.")
            # Pick only the biggest circle by radius
            self.x, self.y, self.r = max(circles, key=lambda c: c[2])
        else:
            self.circle = False
        
    def debug_draw(self):
        if self.circle:
            cv2.circle(self.img, (self.x, self.y), self.r, (0, 255, 0), 2)
            cv2.circle(self.img, (self.x, self.y), 2, (0, 0, 255), 3)  # Center dot
            cv2.putText(self.img, f"({self.x}, {self.y})", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 1)
            cv2.putText(self.img, f"r={self.r}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 1)

        path = 'testimage_circles.jpg'

        cv2.imwrite(path, self.img)
        print(f"Result saved to {path}")


if __name__ == "__main__":
    find_circle = FindCircle()
    find_circle.capture_img()
    find_circle.find_biggest_circle()
    find_circle.debug_draw()