import cv2
import numpy as np

def test_morphological_extraction(input_img):

    h, w, _ = input_img.shape
    k_hor = w // 10
    k_ver = h // 10

    kernel_horizontal = cv2.getStructuringElement(cv2.MORPH_RECT, (k_hor, 1))
    horizontal_structures = cv2.morphologyEx(input_img, cv2.MORPH_OPEN, kernel_horizontal)

    kernel_vertical = cv2.getStructuringElement(cv2.MORPH_RECT, (1, k_ver))
    vertical_structures = cv2.morphologyEx(input_img, cv2.MORPH_OPEN, kernel_vertical)

    junction_img = cv2.bitwise_and(horizontal_structures, vertical_structures)
    return junction_img


def check_branch(binary_img, x0, y0):
    win = 20 # Check window size (needs to be large enough to catch the line)
    branch_min_white_pixels = 50 # Minimum white pixels
    
    x1 = max(x0 - win, 0)
    x2 = min(x0 + win, binary_img.shape[1])
    y1 = max(y0 - win, 0)
    y2 = min(y0 + win, binary_img.shape[0])
    
    if x1 >= x2 or y1 >= y2:
        return False
        
    roi = binary_img[y1:y2, x1:x2]
    return cv2.countNonZero(roi) >= branch_min_white_pixels

def get_junction_center(junction_img):
    MAX_SIZE = 1000000
    MIN_SIZE = 10
    size_check_cnts = []
    sizes = []

    # findContours requires a single-channel binary image
    gray = cv2.cvtColor(junction_img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not cnts:
        return None

    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if MIN_SIZE < area < MAX_SIZE:
            size_check_cnts.append(cnt)
            sizes.append(area)
            
    if not sizes:
        return None

    max_area_index = np.argmax(sizes)
    max_area_cnt = size_check_cnts[max_area_index]
    M = cv2.moments(max_area_cnt)
    if M["m00"] == 0:
        return None, None
        
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    junction_center = cx, cy
    return junction_center


def get_green_mask(frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_a = cv2.inRange(hsv, (35, 40, 40), (85, 255, 255))
        mask_b = cv2.inRange(hsv, (35, 40, 40), (85, 255, 255))
        green_mask = cv2.bitwise_or(mask_a, mask_b)
        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        return green_mask

def find_green_dots(green_mask, output_frame, MIN_GREEN_DOT_AREA, junction_center):
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_dots = []
        for cnt in green_contours:
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            if cv2.contourArea(cnt) > MIN_GREEN_DOT_AREA and cy > junction_center[1]:
                green_dots.append((cx, cy))
                cv2.circle(output_frame, (cx, cy), 3, (255, 0, 255), -1)  # magenta

        return green_dots, output_frame


if __name__ == "__main__":
    # --- Synthetic junction image (no image file needed) ---
    W, H = 500, 500
    cx, cy = W // 2, H // 2   # centre of the junction
    LINE_THICKNESS = 8
    SQUARE_LINES = 20          # half-side of the green square
    DOT_OFFSET = 20           # pixels below each branch tip

    # White background
    frame = np.ones((H, W, 3), dtype=np.uint8) * 255

    branches = [
        (cx, 60),          # top
        (60, cy),          # left
        (W - 60, cy),      # right
        (cx, H - 60),      # bottom
    ]

    for bx, by in branches:
        cv2.line(frame, (cx, cy), (bx, by), (0, 0, 0), LINE_THICKNESS)
    
    cv2.rectangle(frame, (cx + LINE_THICKNESS//2, cy + LINE_THICKNESS//2), (cx + LINE_THICKNESS//2 + SQUARE_LINES, cy + LINE_THICKNESS//2 + SQUARE_LINES), (0, 200, 0), -1)
    cv2.rectangle(frame, (cx - LINE_THICKNESS//2, cy + LINE_THICKNESS//2), (cx - LINE_THICKNESS//2 - SQUARE_LINES, cy +LINE_THICKNESS//2 + SQUARE_LINES), (0, 200, 0), -1)
    cv2.rectangle(frame, (cx + LINE_THICKNESS//2, cy - LINE_THICKNESS//2), (cx + LINE_THICKNESS//2 + SQUARE_LINES, cy -LINE_THICKNESS//2 - SQUARE_LINES), (0, 200, 0), -1)

    original = frame.copy()

    # Run the detection pipeline on the synthetic frame
    junction_img = test_morphological_extraction(frame)
    junction_center = get_junction_center(junction_img)
    green_mask = get_green_mask(frame)
    green_dots, frame = find_green_dots(green_mask, frame, 100, junction_center)
    cv2.circle(frame, junction_center, 3, (255, 0, 0), -1)  # blue
    print(f"Detected green dots: {green_dots}")

    cv2.imshow("Original", original)
    cv2.imshow("Synthetic Junction", cv2.cvtColor(junction_img, cv2.COLOR_BGR2GRAY))
    cv2.imshow("Green Mask", green_mask)
    cv2.imshow("Green Dots", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()