import cv2
import numpy as np

class CentroidFinder:
    def __init__(self):
        self.frame = None
        self.output_frame = None 
        self.MIN_LINE_AREA = 250
        self.MAX_LINE_AREA = 1000
        self.y_levels_amount = 10
        self.angle_deg = 0.0

    def get_threshold(self):
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)    
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=3)
        self.threshold = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=3)

    def get_y_levels(self, height, top_fraction=0.1, bottom_fraction=0.9):

        start = int(height * top_fraction)
        end = int(height * bottom_fraction)

        unique_levels = np.linspace(end, start, self.y_levels_amount, dtype=int)
        self.y_levels = np.unique(unique_levels)[::-1]

    def find_line_centroids(self):
        cx_list = []
        cy_list = []

        for y in self.y_levels:
            y_start = max(0, y - 2)
            y_end = min(self.threshold.shape[0], y + 3)
            slice_line = self.threshold[y_start:y_end, :]
            contours, _ = cv2.findContours(slice_line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.MIN_LINE_AREA and cv2.contourArea(cnt) < self.MAX_LINE_AREA]
                
                if valid_contours:
                    #cv2.drawContours(self.output_frame, valid_contours, -1, (0, 255, 0), 1, offset=(0, y_start))
                    for cnt in valid_contours:
                        M = cv2.moments(cnt)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"]) + y_start
                            cv2.circle(self.output_frame, (cx, int(y)), 5, (0, 255, 0), -1)
                            cx_list.append(cx)
                            cy_list.append(y)
                            break 
        return cx_list

    
    def fit_line_and_angle(self, cx_list):
        x = 0
        dx_sum = 0
        dy_sum = 0

        n = min(len(cx_list), len(self.y_levels))
        if n >= 2:
            while x < n - 1:
                lower_point = cx_list[x]
                upper_point = cx_list[x+1]
                lower_y = self.y_levels[x]
                upper_y = self.y_levels[x+1]

                
                cv2.line(self.output_frame, (int(lower_point), int(lower_y)),
                        (int(upper_point), int(upper_y)), (255, 0, 0), 2)

                delta_y = upper_y - lower_y
                delta_x = upper_point - lower_point
                x = x + 1

                dx_sum += delta_x
                dy_sum += delta_y

            angle_rad = np.arctan2(dx_sum, -dy_sum)
            self.angle_deg = angle_rad * 180.0 / np.pi

            cv2.putText(self.output_frame, f"Angle: {self.angle_deg:.2f}deg", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv2.LINE_AA)


    def draw_line_centroid_distances(self, cx_list, y_levels, width):
            frame_center_x = width // 2
            number = 0
            for cx, y in zip(cx_list, y_levels[:len(cx_list)]):
                number += 1
                distance = cx - frame_center_x
                text = f"{number}: {distance:+d}"
                # Clamp text position to stay inside the frame
                text_x = min(max(cx + 10, 0), width - 40)
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


    def process_image(self, image_path):
        self.frame = cv2.imread(image_path)
        if self.frame is None:
            return
        height, width = self.frame.shape[:2]
        self.output_frame = self.frame.copy()

        self.get_threshold()
        self.get_y_levels(height)
        cx_list = self.find_line_centroids()
        self.fit_line_and_angle(cx_list)
        self.draw_line_centroid_distances(cx_list, self.y_levels, width)

        cv2.imwrite("output.png", self.output_frame)

if __name__ == "__main__":
    finder = CentroidFinder()
    finder.process_image('line.jpg')
