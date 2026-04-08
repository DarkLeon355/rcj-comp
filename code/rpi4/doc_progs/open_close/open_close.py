import cv2
import numpy as np

img_open = cv2.imread("to_open.png")
img_close = cv2.imread("to_close.png")
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))

opened_img = cv2.morphologyEx(img_open, cv2.MORPH_OPEN, kernel, iterations=1)
closed_img = cv2.morphologyEx(img_close, cv2.MORPH_CLOSE, kernel, iterations=1)

cv2.imwrite("to_open_opened.png", opened_img)
cv2.imwrite("to_close_closed.png", closed_img)