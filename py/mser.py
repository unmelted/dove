import cv2
import os
import numpy as np

datapath = ""
#datapath = "py/sample"
#imglist = os.listdir(datapath)
#imglist.sort()

lx = 40
ly = 0
bx = 1800
by = 900

def checkin(rect, index, rectlist) :
    for jj, j in enumerate(rectlist) :
        if jj <= index :
            continue
        print("check in ", rect)
        print("compare " ,j)
        if rect[0] >= j[0] and rect[1] >= j[1] and rect[2] <= j[2] and rect[3] <= j[3] :
            print(" ture")
            return True
        
    print("false")        
    return False
    
def getiou(rect1, index, rectlist) :

    iou = 0.0
    max_iou = 0.0
    print("getiou", rect1)
    for jj, j in enumerate(rectlist) :
        if jj <= index :
            continue

        area1 = j[2] * j[3]
        area2 = rect1[2] * rect1[3]

        inter_x = min(rect1[2] + rect1[0], j[2] + j[0]) - max(rect1[0], j[0])
        inter_y = min(rect1[1] + rect1[3], j[1] + j[3]) - max(rect1[1], j[1])
        print(j)
        print(area1, area2, inter_x, inter_y)
        if inter_x == 0 and inter_y == 0 :
            return 100

        if inter_x > 0 and inter_y > 0 :
            inter_area = inter_x * inter_y
            uni_area = area1 + area2 - inter_area
            iou = inter_area / uni_area
            if iou > max_iou :
                max_iou = iou        

    print("maxiou", max_iou)
    return max_iou

cap = cv2.VideoCapture(datapath+'movie/4dmaker_626.mp4')
index = 0
while(cap.isOpened()):
#for i in imglist :
    #if i == ".DS_Store" :
    #    continue
    print("::::::::: ", index)
    ret, img = cap.read()
    #print(i)
    #img = cv2.imread(datapath+"/"+i)
    img = cv2.resize(img, (1920, 1080))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(3, 3),0)

    mser = cv2.MSER_create(14, 100, 100000, 0.9)
    regions, bboxes = mser.detectRegions(gray)
    rectlist_u = []
    for b in (bboxes) :
        start = (b[0], b[1])
        end = ((b[0] + b[2]), (b[1] + b[3]))
        #print(" b " , b)
        if b[0] >= lx and b[1] >= ly and b[0]+b[2] < bx and (b[1] + b[3]) < by :
            rectlist_u.append(b)

    rectlist = sorted(rectlist_u, key=lambda rectlist_u : rectlist_u[2] * rectlist_u[3])
    print("rect list " ,rectlist)
    clone = img.copy()
    # if len(addlist) > 0 :
    #     hulls = cv2.convexHull(np.array(addlist))
    #     print("hulls ", hulls)
    confirm = []
    for ii, i in enumerate(rectlist) :
        print(ii, i)
        iou = getiou(i, ii, rectlist)
        b_in = checkin(i, ii, rectlist)
        if iou < 0.3 and b_in == False:
            confirm.append(i)

    # if len(confirm) == 0 :
    #     cv2.imwrite("{}.png".format(index), img)

    for c in confirm :
        print(" {} Finally rect list draw {}".format(index, c))
        cv2.rectangle(clone, (c[0], c[1]), (c[0]+c[2],c[1]+c[3]), ( 255,0,255), 2)


    #cv2.drawContours(clone, hulls, -1, (255, 0, 0))
    cv2.imshow("{}".format(index), clone)
    #cv2.waitKey()
    if cv2.waitKey(25) & 0xFF == ord('q') :
        break
    index += 1

cap.release


#    hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in regions]
'''
    remove1 = []
    for i,c1 in enumerate(hulls):

        x, y, w, h = cv2.boundingRect(c1)
        r1_start = (x, y)
        r1_end = (x+w, y+h)

        for j,c2 in enumerate(hulls):
            
            if i == j:
                continue

            x, y, w, h = cv2.boundingRect(c2)
            r2_start = (x, y)
            r2_end = (x+w, y+h)

            if r1_start[0]> r2_start[0] and r1_start[1] > r2_start[1] and r1_end[0] < r2_end[0] and r1_end[1] < r2_end[1]:
                remove1.append(i)


    for j,cnt in enumerate(hulls):
        if j in remove1: continue
        x, y, w, h = cv2.boundingRect(cnt)
        margin = 10
        cv2.rectangle(clone, (x-margin, y-margin), (x + w + margin, y + h + margin), (0, 255, 0), 1)

    cv2.imshow('mser', clone)
    cv2.waitKey(0)


    mask = np.zeros((img.shape[0], img.shape[1], 1), dtype=np.uint8)

    # for j,cnt in enumerate(hulls):
    #     if j in remove1: continue
    #     x, y, w, h = cv2.boundingRect(cnt)
    #     margin = 10
    #     cv2.rectangle(mask, (x-margin, y-margin), (x + w + margin, y + h + margin), (255, 255, 255), -1)
    cv2.rectangle(mask, (80, 200), (1600, 800), (255, 255, 255), -1)
    text_only = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow("text only", text_only)
    cv2.waitKey(0) 
    '''
