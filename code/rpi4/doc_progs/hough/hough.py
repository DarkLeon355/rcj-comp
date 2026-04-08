import cv2
import numpy as np

img = cv2.imread('strasse.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

canny_image = cv2.Canny(gray, 50, 150)

lines = cv2.HoughLinesP(
    canny_image,              
    rho=1,              
    theta=np.pi/180,   
    threshold=100,      
    minLineLength=50,  
    maxLineGap=10       
)

if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

cv2.imshow("Erkannte Linien", img)
cv2.imwrite("lines.png", img)
cv2.waitKey(0)
cv2.destroyAllWindows()