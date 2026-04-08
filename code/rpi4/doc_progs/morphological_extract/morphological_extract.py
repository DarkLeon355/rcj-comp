
import cv2
import numpy as np

def test_morphological_extraction():
    input_img = np.zeros((400, 400), dtype=np.uint8)
    
    #L turn
    #cv2.line(input_img, (50, 300), (200, 300), 255, thickness=10)
    #cv2.line(input_img, (200, 100), (200, 300), 255, thickness=10)

    #Cross
    cv2.line(input_img, (50, 200), (350, 200), 255, thickness=10)
    cv2.line(input_img, (200, 50), (200, 350), 255, thickness=10)

    cv2.imwrite("morph_test_0_input.png", input_img)

    h, w = input_img.shape
    k_hor = w // 10
    k_ver = h // 10

    kernel_horizontal = cv2.getStructuringElement(cv2.MORPH_RECT, (k_hor, 1))
    horizontal_structures = cv2.morphologyEx(input_img, cv2.MORPH_OPEN, kernel_horizontal)

    kernel_vertical = cv2.getStructuringElement(cv2.MORPH_RECT, (1, k_ver))
    vertical_structures = cv2.morphologyEx(input_img, cv2.MORPH_OPEN, kernel_vertical)

    junction = cv2.bitwise_and(horizontal_structures, vertical_structures)

    cv2.imwrite("morph_test_1_horizontal.png", horizontal_structures)
    cv2.imwrite("morph_test_2_vertical.png", vertical_structures)
    cv2.imwrite("morph_test_3_junction.png", junction)

    return junction, input_img


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
    MAX_SIZE = 1000
    MIN_SIZE = 10
    size_check_cnts = []
    sizes = []

    cnts, _ = cv2.findContours(junction_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
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

    

def check_junction(junction_center, junction_check_up_down_left_right, original_img):
    cx, cy = junction_center
    
    offset_up = junction_check_up_down_left_right[0]
    offset_left = junction_check_up_down_left_right[1]
    offset_right = junction_check_up_down_left_right[2]
    offset_down = junction_check_up_down_left_right[3]
    
    check_up_y = cy - offset_up
    check_left = cx - offset_left
    check_right = cx + offset_right
    check_down_y = cy + offset_down

    number_of_branches = 0

    if check_up_y > 0 and check_branch(original_img, cx, check_up_y):
        number_of_branches += 1
        cv2.circle(original_img, (cx, check_up_y), 2, (0, 0, 0), 2)
    if check_left > 0 and check_branch(original_img, check_left, cy):
        number_of_branches += 1
        cv2.circle(original_img, (check_left, cy), 2, (0, 0, 0), 2)
    if check_right < original_img.shape[1] and check_branch(original_img, check_right, cy):
        number_of_branches += 1
        cv2.circle(original_img, (check_right, cy), 2, (0, 0, 0), 2)
    if check_down_y < original_img.shape[0] and check_branch(original_img, cx, check_down_y):
        number_of_branches += 1
        cv2.circle(original_img, (cx, check_down_y), 2, (0, 0, 0), 2)

    if number_of_branches >= 3:
        junction = True
        cv2.circle(original_img, (cx, cy), 2, (0, 0, 0), 2)
    else:
        junction = False
    
    return original_img
        

if __name__ == "__main__":
    junction_joint_img, original_img = test_morphological_extraction()
    junction_center = get_junction_center(junction_joint_img)

    if junction_center:
        cv2.imwrite("morph_test_4_junction_checked.png", check_junction(junction_center, [100, 100, 100, 100], original_img))
    else:
        print("No junction found")

    