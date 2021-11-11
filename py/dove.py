import os
import numpy as np
import cv2


datapath = "image/"
imglist = os.listdir(datapath)
print(imglist)
idx = 0
src1 = None
src2 = None

for i in imglist : 
    name, ext = os.path.splitext(i)
    print("name : {}  ext : {} idx : {} ".format(name, ext, idx))
    if ext == '.png' or ext == '.jpg' :
        src1 = cv2.imread(datapath + "/" + i)
        idx = idx + 1
        if idx == 1 :
            src1 = src2
            continue

        print(idx)
        
        # 그레이스케일로 변환
        gray1 = cv2.cvtColor(src1, cv2.COLOR_BGR2GRAY)
        print("check 2")
        # 코너점 찾는 함수, 그레이스케일 영상만 입력 가능
        pt1 = cv2.goodFeaturesToTrack(gray1, 50, 0.01, 10)
        print("check 3")
        # 찾은 코너점 정보를 옵티컬플로우 함수에 입력
        # src1, src2에서 움직임 정보를 찾아내고 pt1에 입력한 좌표가 어디로 이동했는지 파악
        pt2, status, err = cv2.calcOpticalFlowPyrLK(src1, src2, pt1, None)
        print("check 4")
        # 가중합으로 개체가 어느 정도 이동했는지 보기 위함
        dst = cv2.addWeighted(src1, 0.5, src2, 0.5, 0)
        print("check 5")

        # pt1과 pt2를 화면에 표시
        for i in range(pt2.shape[0]):
            if status[i,0] == 0: # status = 0인 것은 제외, 잘못 찾은 것을 의미
                continue
            print("check 6")        
            cv2.circle(dst, tuple(pt1[i, 0]), 4, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(dst, tuple(pt2[i, 0]), 4, (0, 0, 255), 2, cv2.LINE_AA)
            print("check 7")
            # pt1과 pt2를 이어주는 선 그리기
            cv2.arrowedLine(dst, tuple(pt1[i, 0]), tuple(pt2[i, 0]), (0, 255, 0), 2)

        print("check 8")
        cv2.imwrite("{}_{}.png".format(name, idx), dst)
        src2 = src1

        