import os
import numpy as np
import cv2


file1 = "test.png"

src1 = cv2.imread(file1) #BGR
src2 = cv2.cvtColor(src1, cv2.COLOR_BGR2RGB)
src22 = cv2.cvtColor(src2, cv2.COLOR_RGB2BGR)
cv2.imwrite("RGBA.png", src22)
src3 = cv2.cvtColor(src2, cv2.COLOR_RGB2YUV)
src3 = cv2.cvtColor(src3, cv2.COLOR_YUV2BGR)
cv2.imwrite("YUV.png", src3)
