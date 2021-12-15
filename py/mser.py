import cv2
import os
import numpy as np

datapath = ""
#datapath = "py/sample"
#imglist = os.listdir(datapath)
#imglist.sort()

lx = 5
ly = 5
bx = 630
by = 470

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

#def makeroi() :

class dt() :
    threshold = 0.0
    prev_summ = 0.0
    summ = 0
    def detectsequence(self, bg, cur, index) :
        #cv2.imwrite("{}_cur.png".format(index), cur)    
        cur = cv2.resize(cur, (640, 480))
        cur = cv2.GaussianBlur(cur, (3, 3), 0)
        diff = cv2.subtract(bg_gray, cur)
        # cv2.imwrite("{}_bg.png".format(index), bg_gray)    
        # cv2.imwrite("{}_cur.png".format(index), cur)
        cv2.imwrite("{}_diff.png".format(index), diff)
        self.summ = cv2.sumElems(diff)[0] / (640* 480)
        #print("-- noise summ : " ,self.summ)
        return diff, self.summ
        '''
        if index == 1 :
            self.prev_summ = self.summ
            self.threshold = self.summ * 10
            return False
        else :
            if self.summ - self.prev_summ > self.threshold :
                return True
            else :
                return False
        '''

cap = cv2.VideoCapture(datapath+'movie/4dmaker_598.mp4')
index = 1
ret, bg = cap.read()
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

while(cap.isOpened()):
#for i in imglist :
    #if i == ".DS_Store" :
    #    continue
    print(" -- ", index)
    ret, img = cap.read()
    #print(i)
    #img = cv2.imread(datapath+"/"+i) 
    img = cv2.resize(img, (1920, 1080))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.imwrite("{}_cur.png".format(index), gray)            
    cur = gray.copy()
    #cv2.imwrite("{}_cur.png".format(index), cur)        
    diff, noise = ddd.detectsequence(bg, cur, index)
    #if move == True :
    #     print(":::::::: ------------ swipe ------------- :::::::::::")

    #gray = cv2.GaussianBlur(gray,(3, 3),0)

    mser = cv2.MSER_create(5, 170, 16000, 0.5)
    regions, bboxes = mser.detectRegions(diff)
    rectlist_u = []
    for b in (bboxes) :
        start = (b[0], b[1])
        end = ((b[0] + b[2]), (b[1] + b[3]))
        #print(" b " , b)
        if b[0] >= lx and b[1] >= ly and b[0]+b[2] < bx and (b[1] + b[3]) < by :
            rectlist_u.append(b)

    #rectlist = []
    rectlist = sorted(rectlist_u, key=lambda rectlist_u : rectlist_u[2] * rectlist_u[3])
    # for i in rectlist2 :
    #     if i[2] * i[3] > 200 :
    #         rectlist.append(i)

    #print("rect list " ,rectlist)
    clone = diff.copy()
    # if len(addlist) > 0 :
    #     hulls = cv2.convexHull(np.array(addlist))
    #     print("hulls ", hulls)
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
    if len_con > 0 and found == False and confirm[len_con -1][2] * confirm[len_con -1][2] > 200:
        found = True
        track_obj = confirm[len_con -1]
        track_id = len_con -1
        track_cenx = (confirm[len_con -1][0] + confirm[len_con -1][2]/ 2) 
        track_ceny = (confirm[len_con -1][1] + confirm[len_con -1][3] /2)        
        print(" -------- track start ", track_obj, track_cenx, track_ceny)

    elif len_con > 0 and found == True :
        ffound = False
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



    di = 0
    for c in confirm :
        print(" {} Finally rect list draw {}".format(index, c))
        cv2.rectangle(clone, (c[0], c[1]), (c[0]+c[2],c[1]+c[3]), (255, 0, 255), 2)
        if di == track_id :
            cv2.putText(clone, "FOCUS!", (c[0], c[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,  (255, 255, 40), 1)

        di += 1

    #cv2.drawContours(clone, hulls, -1, (255, 0, 0))
    cv2.imshow("TEST", clone)
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
