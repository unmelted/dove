import os
import cv2
import pathlib
import sys
import numpy as np

path1 = "/Users/4dreplay/work/data/winter/frames/data/2021_03_11_20_40_59_4NylanderGoal/000001.jpg"
path2 = "/Users/4dreplay/work/data/winter/frames/data/2021_03_11_20_40_59_4NylanderGoal/000153.jpg"
src1 = cv2.imread(path1)
src2 = cv2.imread(path2)
# cv2.imshow("TEST", src)
# cv2.waitKey(0)

#gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
#clahe = cv2.createCLAHE(clipLimit=20, tileGridSize=(5,5))
#equ = cv2.equalizeHist(gray)
# equ = clahe.apply(gray)
# cv2.imshow("TEST", equ)
# cv2.waitKey(0)

src1 = cv2.resize(src1, (640, 360))
src2 = cv2.resize(src2, (640, 360))
# src1 = cv2.GaussianBlur(src1, (3, 3), 0)
# src2 = cv2.GaussianBlur(src2, (3, 3), 0)        
diff = cv2.subtract(src1, src2)
cv2.imshow("TEST", diff)
cv2.waitKey(0)
