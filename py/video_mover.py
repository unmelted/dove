import os
import cv2
import pathlib
import numpy as np
import random


# 2021_03_11_20_40_59_4NylanderGoal start = 153 end = 188
# BUT_TATAR 4-0 start = 188 end = 232
# 2018_02_09_17_58_50_0.mp4_stab start = 64 end = 88
#2018_02_25_09_55_28.mp4_stab start = 58 end = 91
out = cv2.VideoWriter('test_out.mp4', cv2.VideoWriter_fourcc(*"MJPG"), 30,(1920, 1080))
datapath = "/Users/4dreplay/work/data/winter/frames/data/2018_02_25_09_55_28.mp4_stab"
imglist = os.listdir(datapath)
imglist.sort()

start = 58
end = 91

want_cnt = 10
index = 0
for i in imglist : 
    if i == ".DS_Store" or i == "Thumbs.db": 
        continue
    
    filename = datapath +"/"+ i
    imgsrc = cv2.imread(filename)
    print(i, filename)

    if index <= start or index >= end :
        out.write(imgsrc)
        index += 1
        continue

#    for j in range(0, want_cnt) :

    shift_x = random.uniform(-15, 15)
    shift_y = random.uniform(-15, 15)
#        brightness = random.uniform(0.4, 2)
#        print(filename)
#        print("random shift ", shift_x, shift_y)
    M = np.float32([[1, 0, shift_x], [0, 1, shift_y]])
    sftimg = cv2.warpAffine(imgsrc, M, (imgsrc.shape[1], imgsrc.shape[0]))
    out.write(sftimg)
    index += 1
#        gamma = 1.0 / brightness
#        table = np.array([((i / 255.0) ** gamma) * 255
#		for i in np.arange(0, 256)]).astype("uint8")
#        final = cv2.LUT(sftimg, table)
#        newname = filename[:-4] + "_" + str(j)+ "_sfhit_"+ str(shift_x)[:6] + "_" + str(shift_y)[:6] + ".png"
#        print(newname)
#        cv2.imwrite(newname, final)
out.release()