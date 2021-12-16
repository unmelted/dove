import cv2
import os
import numpy as np
import timeit

datapath = ""
#datapath = "py/sample"
#imglist = os.listdir(datapath)
#imglist.sort()

lx = 5
ly = 5
bx = 630
by = 470
roi_sx = 0
roi_sy = 0
roi_w = 400
roi_h = 300
scale = 6

proi_sx = 0
proi_sy = 0
proi_w = 0
proi_h = 0

def checkin(rect, index, rectlist) :
    for jj, j in enumerate(rectlist) :
        if jj <= index :
            continue
#        print("check in ", rect)
#        print("compare " ,j)
        if rect[0] >= j[0] and rect[1] >= j[1] and rect[2] <= j[2] and rect[3] <= j[3] :
#            print(" ture")
            return True
        
#    print("false")        
    return False
    
def getiou(rect1, index, rectlist) :

    iou = 0.0
    max_iou = 0.0
#    print("getiou", rect1)
    for jj, j in enumerate(rectlist) :
        if jj <= index :
            continue

        area1 = j[2] * j[3]
        area2 = rect1[2] * rect1[3]

        inter_x = min(rect1[2] + rect1[0], j[2] + j[0]) - max(rect1[0], j[0])
        inter_y = min(rect1[1] + rect1[3], j[1] + j[3]) - max(rect1[1], j[1])
        #print(j)
        #print(area1, area2, inter_x, inter_y)
        if inter_x == 0 and inter_y == 0 :
            return 100

        if inter_x > 0 and inter_y > 0 :
            inter_area = inter_x * inter_y
            uni_area = area1 + area2 - inter_area
            iou = inter_area / uni_area
            if iou > max_iou :
                max_iou = iou        

    #print("maxiou", max_iou)
    return max_iou

def makeroi(track_cenx, track_ceny, w, h) :
    min_w = 400
    min_h = 300
    sx = 0
    sy = 0
    real_w = max(min_w, w)
    real_h = max(min_h, h)

    if track_cenx + real_w /2 > bx :
        real_w = real_w - ((track_cenx + real_w /2) - bx)
    if track_ceny + real_h /2 > by :
        real_h = real_h - ((track_ceny + real_h /2) - by)

    if track_cenx - (real_w /2) > lx :
        roi_sx = track_cenx - (real_w /2)
    else :
        roi_sx = lx
    if track_ceny - (real_h /2 ) > ly :
        roi_sy = track_ceny - (real_h /2 )
    else :
        roi_sy = ly

    print("roi    ", int(roi_sx), int(roi_sy), int(real_w), int(real_h))
    return int(roi_sx), int(roi_sy), int(real_w), int(real_h)


class dt() :
    threshold = 15
    first_summ = 0
    prev_summ = 0
    prev_diff = 0
    summ = 0
    peak_summ = 0
    isSwipe = False
    normal = 0;
    bgcg = None
    skip = 100

    def detectsequence(self, bg, cur, index) :
        #cv2.imwrite("{}_cur.png".format(index), cur)    
        cur = cv2.resize(cur, (640, 480))
        cur = cv2.GaussianBlur(cur, (3, 3), 0)
        diff = None
        # if self.isSwipe == True : 
        #     diff = cv2.subtract(self.bgcg, cur)                    
        # else : 
        diff = cv2.subtract(bg, cur)        

        self.summ = cv2.sumElems(diff)[0] / (640* 480)
        
        # if self.isSwipe == True and self.skip > 70 :
        #     self.summ = 10
        #     self.skip -= 1
        #     self.bgcg = cur.copy()

        if index == 1 :
            self.first_summ = self.summ
        else :
            if self.isSwipe == False :
                if self.summ - self.prev_summ > self.first_summ * self.threshold :
                    self.isSwipe = True
                    #self.bgcg = cur.copy()                    
                else :
                    self.isSwipe = False

            elif self.isSwipe == True :
                if self.summ <= 2: 
                    self.isSwipe = False
                else :
                    self.isSwipe = True

        self.prev_diff = self.summ - self.prev_summ
        self.prev_summ = self.summ
        return diff, self.summ, self.isSwipe

class KalmanFilter() :
    F = None
    B = None
    H = None
    Q = None
    R = None
    P = None
    x0 = None
    def init(self, F, H, Q , R) :
        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n)
        self.x = np.zeros((self.n, 1))

    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)


out = cv2.VideoWriter('test_out.avi', cv2.VideoWriter_fourcc(*"MJPG"), 30,(1920, 1080))
cap = cv2.VideoCapture(datapath+'movie/4dmaker_626.mp4')
index = 1
ret, bg = cap.read()
prevc = bg.copy()
prevc = cv2.resize(prevc, (1920, 1080))
prev =cv2.cvtColor(prevc, cv2.COLOR_BGR2GRAY)
bg = cv2.resize(bg, (640, 480))
bg_gray = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY)
bg_gray = cv2.GaussianBlur(bg_gray,(3, 3),0)
prev_summ = 0
threshold = 0    
found = False
ddd = dt()
track_id = 0
track_obj = 0
track_cenx = 0
track_ceny = 0
ptrack_cenx = 0
ptrack_ceny = 0
mask = 0
curc = None
ismove = False
warp_dst = None

dt = 1.0/60
F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H = np.array([1, 0, 0]).reshape(1, 3)
Q = np.array([[0.0001, 0.0001, 0.0], [0.0001, 0.0001, 0.0], [0.0, 0.0, 0.0]])
R = np.array([0.2]).reshape(1, 1)
kfx = KalmanFilter()
kfx.init(F, H, Q, R)
kfy = KalmanFilter()
kfy.init(F, H, Q, R)

while(cap.isOpened()):
    print(" -- ", index)
    start_time = timeit.default_timer()    
    ret, img = cap.read()
    if ret == False :
        break

    img = cv2.resize(img, (1920, 1080))
    curc = img
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite("saved/{}_cur.png".format(index), curc)            
    cur = gray.copy()
    #cv2.imwrite("{}_cur.png".format(index), cur)        
    diff, noise, ismove = ddd.detectsequence(bg_gray, cur, index)
    # if ismove == True :
    #     print(":::::::: ------------ swipe ------------- :::::::::::", noise)
    # else : 
    #     print("------------ no -------------", noise)

    dst = diff
    mser = cv2.MSER_create(5, 170, 16000, 0.5)
     #gray = cv2.GaussianBlur(gray,(3, 3),0)
    if found == True : 
        mask = np.zeros((480, 640), dtype=np.uint8)        
        cv2.rectangle(mask, (roi_sx, roi_sy), (roi_sx + roi_w, roi_sy + roi_h), (255), -1)
        dst = cv2.bitwise_and(diff, mask)
        #cv2.imwrite("{}_bitwise.png".format(index), dst)

    regions, bboxes = mser.detectRegions(dst)
    rectlist_u = []
    for b in (bboxes) :
        start = (b[0], b[1])
        end = ((b[0] + b[2]), (b[1] + b[3]))
        #print(" b " , b)
        if b[0] >= lx and b[1] >= ly and b[0]+b[2] < bx and (b[1] + b[3]) < by :
            rectlist_u.append(b)

    rectlist = sorted(rectlist_u, key=lambda rectlist_u : rectlist_u[2] * rectlist_u[3])
    clone = diff.copy()    
    confirm = []
    confirm2 = []

    for ii, i in enumerate(rectlist) :
        b_in = checkin(i, ii, rectlist)
        if b_in == False:
            confirm2.append(i)

    for ii, i in enumerate(confirm2) :
        #print(ii, i)
        iou = getiou(i, ii, confirm2)
        if iou < 0.3 :
            confirm.append(i)

    #print("before confirm.. ")
    print(confirm)
    len_con = len(confirm)
    ffound = False    
    if len_con > 0 and found == False and confirm[len_con -1][2] * confirm[len_con -1][2] > 200:
        found = True
        track_obj = confirm[len_con -1]
        track_id = len_con -1
        track_cenx = (confirm[len_con -1][0] + confirm[len_con -1][2]/ 2) 
        track_ceny = (confirm[len_con -1][1] + confirm[len_con -1][3] /2)        
        print(" -------- track start ", track_obj, track_cenx, track_ceny)

    elif len_con > 0 and found == True :
        for ii, i in enumerate(confirm) :
            #if abs(i[0] - track_obj[0]) < 10 and abs(i[1] - track_obj[1]) < 10 and abs(i[2] - track_obj[2]) < 15 and abs(i[3] - track_obj[3]) < 15 :
            icen_x = i[0] + i[2] /2
            icen_y = i[1] + i[3] /2
            if abs(icen_x - track_cenx) < 30 and abs(icen_y - track_ceny) < 30 :
                track_id = ii
                track_obj = i
                track_cenx = icen_x
                track_ceny = icen_y
                ffound = True
                print(" keep tracking ", track_obj, track_cenx, track_ceny)
                break

        if ffound == False :
            print(" missed !! ")
            found = False
            #break

    if found == True :
        roi_sx, roi_sy, roi_w, roi_h  = makeroi(track_cenx ,track_ceny, track_obj[2], track_obj[3])

    di = 0
    for c in confirm :
        print(" {} Finally rect list draw {}".format(index, c))
        if di == track_id and found == True :
            cv2.rectangle(clone, (c[0], c[1]), (c[0]+c[2],c[1]+c[3]), (255, 0, 255), 2)            
            cv2.putText(clone, "FOCUS!", (c[0], c[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,  (255, 255, 40), 1)
            cv2.rectangle(clone, (roi_sx, roi_sy), (roi_sx + roi_w, roi_sy + roi_h), (255), 2)
            cv2.imwrite("black/{}_clone.png".format(index), clone)            
        di += 1

    if ffound == True :
        if ismove == 3:
            #cv2.imwrite("{}_prev.png".format(index), prev)
            cur_crop = cur[roi_sy * 3 : (roi_sy + roi_h) * 3, roi_sx * 3 : (roi_sx + roi_w) * 3]
            prev_crop = prev[roi_sy * 3 : (roi_sy + roi_h) * 3, roi_sx * 3 : (roi_sx + roi_w) * 3]
            cv2.imwrite("{}_curcrop1.png".format(index), cur_crop)
            cv2.imwrite("{}_prevcrop1.png".format(index), prev_crop)        
            pt1 = cv2.goodFeaturesToTrack(prev_crop, 200, 0.01, 10)
            pt2, status, err = cv2.calcOpticalFlowPyrLK(prev_crop, cur_crop, pt1, None)
            gdpt1 = []
            gdpt2 = []
            for i in range(pt2.shape[0]) :
                gdpt1.append(pt1[i])
                gdpt2.append(pt2[i])
            
            warp_m, _ = cv2.estimateAffine2D(np.array(gdpt1).astype(np.float32), np.array(gdpt2).astype(np.float32))
            warp_m[0][0] = 1
            warp_m[0][1] = 0
            warp_m[1][0] = 0        
            warp_m[1][1] = 1  
            warp_m[0][2] = warp_m[0][2]
            warp_m[1][2] = warp_m[1][2]
            print("warp matrix  1", warp_m[0][2], warp_m[1][2])
            warp_dst = cv2.warpAffine(prevc, warp_m, (1920,1080))
        else : 
            #cv2.imwrite("{}_prev.png".format(index), prev)            
            #cur_crop = cur[roi_sy * 3 : (roi_sy + roi_h) * 3, roi_sx * 3 : (roi_sx + roi_w) * 3]
            #prev_crop = prev[roi_sy * 3 : (roi_sy + roi_h) * 3, roi_sx * 3 : (roi_sx + roi_w) * 3]
            #cv2.imwrite("{}_curcrop2.png".format(index), cur_crop)
            #cv2.imwrite("{}_prevcrop2.png".format(index), prev_crop)        
            pt1 = cv2.goodFeaturesToTrack(prev, 200, 0.01, 10)
            pt2, status, err = cv2.calcOpticalFlowPyrLK(prev, cur, pt1, None)
            gdpt1 = []
            gdpt2 = []
            for i in range(pt2.shape[0]) :
                gdpt1.append(pt1[i])
                gdpt2.append(pt2[i])
            
            warp_m, _ = cv2.estimateAffine2D(np.array(gdpt1).astype(np.float32), np.array(gdpt2).astype(np.float32))
            kfx.update((track_cenx - ptrack_cenx) *3)
            kfy.update((track_ceny - ptrack_ceny) *3)                 
            px = (np.dot(H,  kfx.predict())[0])
            py = (np.dot(H,  kfy.predict())[0])
            warp_m[0][0] = 1
            warp_m[0][1] = 0
            warp_m[1][0] = 0        
            warp_m[1][1] = 1  
            warp_m[0][2] = px * 0.3 #track_cenx - ptrack_cenx#warp_m[0][2]
            warp_m[1][2] = py * 0.3#track_ceny - ptrack_ceny#warp_m[1][2]
            print("warp matrix 2 ", warp_m[0][2], warp_m[1][2])                        
            cv2.imshow("TEST", warp_dst)

            if track_cenx - ptrack_cenx == 0:
                px = (np.dot(H,  kfx.predict())[0])
                py = (np.dot(H,  kfy.predict())[0])
                warp_m[0][0] = 1
                warp_m[0][1] = 0
                warp_m[1][0] = 0        
                warp_m[1][1] = 1  
                warp_m[0][2] = px * 0.3 #track_cenx - ptrack_cenx#warp_m[0][2]
                warp_m[1][2] = py * 0.3#track_ceny - ptrack_ceny#warp_m[1][2]            
            #kfx.update(warp_m[0][2])
            #kfy.update(warp_m[1][2])
            #curc_crop = curc[roi_sy * 3 : (roi_sy + roi_h) * 3, roi_sx * 3 : (roi_sx + roi_w) * 3]            
            warp_dst = cv2.warpAffine(prevc, warp_m, (1920,1080))
            #cur = cv2.warpAffine(cur, warp_m, (1920,1080))            
            cv2.imshow("TEST", warp_dst)
            
            px = (np.dot(H,  kfx.predict())[0])
            py = (np.dot(H,  kfy.predict())[0])
            warp_m[0][0] = 1
            warp_m[0][1] = 0
            warp_m[1][0] = 0        
            warp_m[1][1] = 1  
            warp_m[0][2] = px * 0.3#track_cenx - ptrack_cenx#warp_m[0][2]
            warp_m[1][2] = py * 0.3#track_ceny - ptrack_ceny#warp_m[1][2]            
            #kfx.update(warp_m[0][2])
            #kfy.update(warp_m[1][2])
            #curc_crop = curc[roi_sy * 3 : (roi_sy + roi_h) * 3, roi_sx * 3 : (roi_sx + roi_w) * 3]            
            warp_dst = cv2.warpAffine(prevc, warp_m, (1920,1080))            
    else :
        warp_dst = prevc.copy() 

    terminate_time = timeit.default_timer()
    print("TIME : %f s" % (terminate_time - start_time))
    out.write(warp_dst)

    cv2.imwrite("{}_warp.png".format(index), warp_dst)
    prevc = curc.copy()
    prev = cur.copy()
    proi_sx = roi_sx
    proi_sy = roi_sy
    proi_w = roi_w
    proi_h = roi_h
    ptrack_cenx = track_cenx
    ptrack_ceny = track_ceny

    cv2.imshow("TEST", warp_dst)
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
