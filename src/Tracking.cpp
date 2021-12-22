/*****************************************************************************
*                                                                            *
*                           Tracking.cpp     								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Tracking.cpp
    Author(S)       : Me Eunkyung
    Created         : 17 dec 2021

    Description     : Tracking.cpp
    Notes           : Tracking
*/

#include "Tracking.hpp"
#include <algorithm>
#include <functional>

Tracking::Tracking() {
    p = new PARAM();    
    dl = Dlog();
    isfound = false;
    issame = false;
    first_summ = 0.0;
    ms = MSER::create(3, 170, 16000, 0.5);
    rect_roi = Rect();
}

Tracking::~Tracking() {

}

void Tracking::SetInitialData(PARAM* _p) {
    memcpy(p, _p, sizeof(PARAM));

    if(p->tracker_type == MIL)
        tracker = TrackerMIL::create();
    else if (p->tracker_type == KCF)
        tracker = TrackerKCF::create();
    // else if (p->tracker_type == TLD)
    //     tracker = TrackerTLD::create();
    // else if (p->tracker_type == MEDIANFLOW)
    //     tracker = TrackerMedianFlow::create();
    else if (p->tracker_type == GOTURN)
        tracker = TrackerGOTURN::create();
    // else if (p->tracker_type == MOSSE)
    //     tracker = TrackerMOSSE::create();
    else if (p->tracker_type == CSRT) {
        TrackerCSRT::Params tckp = TrackerCSRT::Params();
        tckp.use_hog = true;    
        tckp.use_gray = true;
        tracker = TrackerCSRT::create();        
    }
}

void Tracking::SetBg(Mat& src, int frame_id) {    
    int histbin = 256;
    double minval; double maxval;
    double cut_threshold;
    Point minloc; Point maxloc;
    Mat hist;
    start_frame = frame_id;

    if(p->track_scale != 1 ) {
        scale_w = int(src.cols/p->track_scale);
        scale_h = int(src.rows/p->track_scale);
        resize(src, bg, Size(scale_w, scale_h));
    }    
    calcHist(&bg, 1, 0, Mat(), hist, 1, &histbin, 0);
    // for (int i = 0 ; i < histbin; i ++){
    //     printf(" [%d] hist %d \n", i , (int)hist.at<float>(i));
    // }

    minMaxLoc(hist, &minval, &maxval, &minloc, &maxloc, Mat());
    printf("searched minval %f maxval %f minloc %d %d maxloc %d %d", minval, maxval, minloc.x, minloc.y, maxloc.x, maxloc.y);
    cut_threshold = maxloc.y * 0.83;
    lut = Mat::zeros(1, histbin, CV_8UC1);
    for (int i = 0 ; i < histbin; i ++){
        if(i <= cut_threshold) 
            lut.at<unsigned char>(i) = i;
        else 
            lut.at<unsigned char>(i) = 255;

//        printf(" [%d] lut--  %d \n", i , lut.at<unsigned char>(i));            
    }

    Mat result; Mat temp;
    LUT(bg, lut, result);

    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(20);
    clahe->apply(result, temp);
    GaussianBlur(temp, bg, {3, 3}, 1.3, 1.3);
    dl.Logger("Setbg function finish %d %d ", bg.cols, bg.rows);
}

bool Tracking::CheckWithin(Rect& r) {
    if(r.x >= p->limit_lx && r.y >= p->limit_ly &&
    (r.x + r.width ) <= p->limit_bx && (r.y + r.height) <= p->limit_by)
        return true;
    else 
        return false;
}

bool Tracking::CheckWithin(Rect& r, int index, vector<Rect>& rects) {
    for(int i = 0 ; i < rects.size(); i++) {
        if( i<= index)
            continue;
        //printf("checkwithin [%d] rects %d %d %d %d \n", i, rects[i].x, rects[i].y, rects[i].width, rects[i].height);            
        if( r.x >= rects[i].x && r.y >= rects[i].y &&
            r.width <= rects[i].width && r.height <= rects[i].height)
            return true;
    }
    return false;
}

void Tracking::ConvertToRect(TRACK_OBJ* roi) {
    rect_roi.x = roi->sx;
    rect_roi.y = roi->sy;
    rect_roi.width = roi->w;
    rect_roi.height = roi->h;
}

void Tracking::ConvertToROI(Rect& rec, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    roi->sx = rec.x;
    roi->sy = rec.y;
    roi->w = rec.width;
    roi->h = rec.height;

    obj->sx = roi->sx + 10;
    obj->sy = roi->sy + 10;
    obj->w = roi->w - 20;
    obj->h = roi->h - 20;

    roi->update();
    obj->update();
}

int Tracking::PickArea(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    int result = 0;
    double minval; double maxval;
    Point minloc; Point maxloc;
    Mat cur; Mat dst;
    ImageProcess(src, cur);
    dl.Logger("PickArea cos/row %d %d st_frame %d index %d", cur.cols, cur.rows, start_frame, index);
    subtract(bg, cur, diff);
    float diff_val = sum(diff)[0]/(scale_w * scale_h);

    minMaxLoc(diff, &minval, &maxval, &minloc, &maxloc, Mat());
    dl.Logger("PickArea minval %f maxval %f minloc %d %d maxloc %d %d", minval, maxval, minloc.x, minloc.y, maxloc.x, maxloc.y);

    obj->update(maxloc.x -30, maxloc.y -30, 60, 90);
    obj->update();
    roi->update(obj->sx - 10, obj->sy - 10, obj->w + 20, obj->h + 20);    
    roi->update();
    dl.Logger("obj %d %d %d %d", obj->sx, obj->sy ,obj->w , obj->h);
    dl.Logger("roi %d %d %d %d", roi->sx, roi->sy ,roi->w , roi->h);
    ConvertToRect(roi);

    tracker->init(diff, rect_roi);
    isfound = true;
    //DrawObjectTracking(diff, obj, roi, false, 1);
}

int Tracking::TrackerUpdate(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    Mat cur; Mat dst;
    ImageProcess(src, cur);
    dl.Logger("TrackerUpdate cos/row %d %d st_frame %d index %d", cur.cols, cur.rows, start_frame, index);
    subtract(bg, cur, diff);
    float diff_val = sum(diff)[0]/(scale_w * scale_h);

    if(index > start_frame +1 && !prev.empty()) {
        Mat same;        
        subtract(prev, cur, same);
        float same_check = sum(same)[0]/(scale_w * scale_h);
        dl.Logger("same check %f ", same_check);
        if (same_check < 0.2) {
            dl.Logger("Current image is same as previous.. ");
            issame = true;
            //return same_check;
        }
        else
            issame = false;    
    }
    else if( index == start_frame ) {
        first_summ = diff_val;    
        dl.Logger("First summ save %f ", first_summ);
    }
    cur.copyTo(prev);

    bool ret = tracker->update(diff, rect_roi);
    if (ret == false) {
        dl.Logger("tracker miss --------------------------------------------");
//        tracker->init(diff, rect_roi);            
    }

    ConvertToROI(rect_roi, obj, roi);
    isfound = true;    
    //DrawObjectTracking(diff, obj, roi, false, 1);
    tracker->init(diff, rect_roi);                    
}

float Tracking::DetectAndTrack(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    int result = 0;
    Mat cur; Mat dst;
    ImageProcess(src, cur);
    dl.Logger("DetectAndTrack function start %d %d st_frame %d index %d", cur.cols, cur.rows, start_frame, index);
    subtract(bg, cur, diff);
    float diff_val = sum(diff)[0]/(scale_w * scale_h);

    if(index > start_frame +1 && !prev.empty()) {
        Mat same;        
        subtract(prev, cur, same);
        // sprintf(filename, "saved/%d_samecheck.png", index);
        // imwrite(filename, same);
        float same_check = sum(same)[0]/(scale_w * scale_h);
        dl.Logger("same check %f ", same_check);
        if (same_check < 0.2) {
            dl.Logger("Current image is same as previous.. ");
            issame = true;
            return same_check;
        }
        else
            issame = false;    
    }
    else if( index == start_frame ) {
        first_summ = diff_val;    
        dl.Logger("First summ save %f ", first_summ);
    }
    cur.copyTo(prev);

    if(isfound == true) {
        Mat mask = Mat::zeros(scale_h, scale_w, CV_8UC1);
        rectangle(mask, Point(roi->sx, roi->sy), Point(roi->ex, roi->ey), Scalar(255), -1);
        bitwise_and(mask, diff, dst);
    }
    else 
        dst = diff;

    prev_diff = diff_val;
    vector<vector<Point>>res_pt;
    vector<Rect>res_rect;
    vector<Rect>t;    
    vector<Rect>q; 

    ms->detectRegions(dst, res_pt, res_rect);
    dl.Logger("DetectRegion result rect count %d", res_rect.size());

    for(int i = 0; i < res_rect.size() ; i ++ ) {
        if(CheckWithin(res_rect[i]) == true)
            t.push_back(res_rect[i]);
    }
    dl.Logger("check within 1st step. t size %d", t.size());
    //sort(t.begin(), t.end(), Compare2);
    // for(int i = 0 ; i < t.size(); i++) 
    //     printf("rect t[i] %d %d %d %d\n", t[i].x, t[i].y, t[i].width, t[i].height);

    std::sort(t.begin(), t.end(), [](Rect a, Rect b) {
              return a.width * a.height < b.width * b.height;;
    });
    for(int i = 0 ; i < t.size(); i++){
        if(CheckWithin(t[i], i, t) == false)
            q.push_back(t[i]);
    }
    dl.Logger("check within 2nd step. q size %d", q.size());
    res_rect.clear();    
    for (int i = 0 ; i < q.size(); i ++) {
        if(GetIOU(q[i], i, q) < p->iou_threshold) {
            res_rect.push_back(q[i]);
        }
    }
    dl.Logger("Filtered rect count %d", res_rect.size());    
    //DrawObjectTracking(obj, roi, res_rect);
    
    int last = res_rect.size() - 1;
    if(res_rect.size() == 0) {
        isfound = false;
        p->keep_tracking = false;
        dl.Logger("Filtered none..");
    }
    else if (res_rect.size() > 0 && isfound == false && res_rect[last].width * res_rect[last].height > p->area_threshold) {
        isfound = true;
        obj->update(res_rect[last].x, res_rect[last].y, res_rect[last].width, res_rect[last].height);
        obj->id = last;
        obj->update();
        dl.Logger("[%d] Object found. obj s %d %d w %d %d c %f %f e %d %d ", index, obj->sx, obj->sy, obj->w, obj->h, obj->cx, obj->cy, obj->ex, obj->ey);
    }
    else if (res_rect.size() > 0 && isfound == true) {
        bool newfound = false;
        for(int i = 0 ; i < res_rect.size(); i ++){
            float tcen_x = res_rect[i].x + res_rect[i].width/2;
            float tcen_y = res_rect[i].y + res_rect[i].height/2;
            if (abs(tcen_x - obj->cx) < p->center_threshold && abs(tcen_y - obj->cy) < p->center_threshold) {
                obj->id = i;
                obj->update(res_rect[i].x, res_rect[i].y, res_rect[i].width, res_rect[i].height);
                obj->update();
                newfound = true;
                break;
            }
        }
        if(newfound == true){
            isfound = true;
            p->keep_tracking = true;
            dl.Logger("[%d] [OBJ] s %d %d w %d %d c %f %f e %d %d", index, obj->sx, obj->sy, obj->w, obj->h, obj->cx, obj->cy,
                obj->ex, obj->ey);
        } else {
            isfound = false;
            p->keep_tracking = false;
            dl.Logger("[%d] ------------------missed !!", index);            
            dl.Logger("[%d] Mssing obj s %d %d w %d %d ", index, obj->sx, obj->sy, obj->w, obj->h);            
        }
    }

    if (isfound == true)
        MakeROI(obj, roi);

    dl.Logger("[%d] isfound %d issmae %d diff_val %f ", index, isfound, issame, diff_val);
    q.clear();
    t.clear();
    return diff_val;
}
    
void Tracking::MakeROI(TRACK_OBJ* obj, TRACK_OBJ* roi) {
    int sx = 0;
    int sy = 0;
    float real_w = max(p->roi_w, obj->w);
    float real_h = max(p->roi_h, obj->h);

    if ((obj->cx + real_w /2) > p->limit_bx) {
        real_w = real_w - ((obj->cx + real_w /2) - p->limit_bx);
        printf("1 real_w %f\n", real_w);
    }
    if ((obj->cx - real_w /2) < p->limit_lx) {
        real_w = real_w - (p->limit_lx - (obj->cx - real_w /2));
        printf("2 real_w %f \n", real_w);        
    }
    if ((obj->cy + real_h /2) > p->limit_by) {
        real_h = real_h - ((obj->cy + real_h /2) - p->limit_by);
        printf("3 real_h %f \n", real_h);
    }
    if ((obj->cy - real_h/ 2) < p->limit_ly) {
        real_h = real_h - (p->limit_ly - (obj->cy - real_h /2));
        printf("4 real_h %f \n", real_h);        
    }

    dl.Logger("modifed ROI w %f h %f", real_w, real_h);

    if ((obj->cx - real_w /2) > p->limit_lx) {
        roi->sx = int(obj->cx - (real_w /2));
        printf("roi->sx %d \n", roi->sx);        
    }
    else 
        roi->sx = p->limit_lx;

    if ((obj->cy - real_h /2) > p->limit_ly) {
        roi->sy = int(obj->cy - (real_h / 2));
        printf("roi->sy %d \n", roi->sy);
    }
    else
        roi->sy = p->limit_ly;

    if ((obj->cx + real_w /2) > p->limit_bx) 
        roi->ex = p->limit_bx;
    else 
        roi->ex = int(obj->cx + real_w /2);
    
    if ((obj->cy + real_h /2) > p->limit_by)
        roi->ey = p->limit_by;
    else 
        roi->ey = int(obj->cy + real_h /2);
    

    dl.Logger("[ROI] s %d %d w %f %f e %d %d", int(roi->sx), int(roi->sy), int(roi->w), int(roi->h),
         int(roi->ex), int(roi->ey));

}

float Tracking::GetIOU(Rect& r, int index, vector<Rect>& rects) {
    float iou = 0.0;
    float max_iou = 0.0;
    for(int i = 0; i < rects.size(); i ++) {
        if(i == index)
            continue;

        float area1 = rects[i].width * rects[i].height;;
        float area2 = r.width * r.height;

        int inter_x = min(r.x + r.width, rects[i].x + rects[i].width) - max(r.x, rects[i].x);
        int inter_y = min(r.y + r.height, rects[i].y +  rects[i].height) - max(r.y, rects[i].y);

        if (inter_x == 0 && inter_y == 0)
            return 100;
        if (inter_x > 0 && inter_y > 0) {
            int inter_area = inter_x * inter_y;
            int uni_area = area1 + area2 - inter_area;
            iou = inter_area / uni_area;
            if(iou > max_iou)
                max_iou = iou;
        }
    }
    return max_iou;
}

void Tracking::ImageProcess(Mat& src, Mat& dst) {
    Mat temp;
    Mat result;
    src.copyTo(temp);

    if(p->track_scale != 1 ) {
        scale_w = int(src.cols/p->track_scale);
        scale_h = int(src.rows/p->track_scale);
        resize(temp, temp, Size(scale_w, scale_h));
//        imwrite("check2.png", dst);
    }
    LUT(temp, lut, result);

    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(20);
    clahe->apply(result, temp);
    GaussianBlur(temp, dst, {5, 5}, 1.3, 1.3);
}

void Tracking::DrawObjectTracking(Mat& src, TRACK_OBJ* obj, TRACK_OBJ* roi, bool borigin, int replay_style) {
    Mat canvas;
    if(borigin == true) 
        src.copyTo(canvas);
    else
        diff.copyTo(canvas);

    string contents = "";
    switch(replay_style) {
        case KEEP_TRACKING_SWIPE:
            contents = "Keep tracking on swipe";
        case SWIPE_ON :
            contents = "Swipe on";
        case SWIPE_END :
            contents = "Swipe off";
        case MISSED_TRACKING :
            contents = "Missed object";
        case TRACK_NONE :
            contents = "None ";
    }    
    if(issame)
        contents += " SAME";
        
    if(isfound == true){ 
        if(borigin == false) {
            rectangle(canvas, Point(obj->sx, obj->sy), Point(obj->ex, obj->ey), (255), 2);
            putText(canvas, "FOCUS", Point(obj->sx, obj->sy - 10), FONT_HERSHEY_SIMPLEX, 0.4, (255), 1);
            rectangle(canvas, Point(roi->sx, roi->sy), Point(roi->ex, roi->ey), (255), 2);
            putText(canvas, contents, Point(10, 10), FONT_HERSHEY_SIMPLEX, 0.4, (255), 1);            
        } else if(borigin == true){
            rectangle(canvas, Point(obj->sx * p->track_scale, obj->sy * p->track_scale), Point(obj->ex * p->track_scale, obj->ey * p->track_scale), (0), 2);
            putText(canvas, "FOCUS", Point(obj->sx * p->track_scale, (obj->sy - 20) * p->track_scale), FONT_HERSHEY_SIMPLEX, 0.6, (0), 1);
            rectangle(canvas, Point(roi->sx* p->track_scale, roi->sy * p->track_scale), Point(roi->ex * p->track_scale, roi->ey * p->track_scale), (0), 2);                
            putText(canvas, contents, Point(10, 10), FONT_HERSHEY_SIMPLEX, 0.4, (0), 1);
        }
    }
    imshow("FIRST PROCESS", canvas);
    waitKey(0);    
}

void Tracking::DrawObjectTracking(TRACK_OBJ* obj, TRACK_OBJ* roi, vector<Rect> rects) {
    Mat canvas;
    diff.copyTo(canvas);

    for(int i = 0 ; i < rects.size(); i ++) {
        rectangle(canvas, Point(rects[i].x, rects[i].y), Point(rects[i].x + rects[i].width, rects[i].y + rects[i].height), (255), 2);
    }

    putText(canvas, "FOCUS", Point(obj->sx, obj->sy - 10), FONT_HERSHEY_SIMPLEX, 0.4, (255), 1);
    rectangle(canvas, Point(roi->sx, roi->sy), Point(roi->ex, roi->ey), (255), 2);

    imshow("FIRST PROCESS", canvas);
    waitKey(0);    
}