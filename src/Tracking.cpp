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
    ms = MSER::create(5, 170, 16000, 0.8);
    p = new PARAM();    
    dl = Dlog();
    isfound = false;
    issame = false;
    first_summ = 0.0;
}

Tracking::~Tracking() {

}

void Tracking::SetBg(Mat& src) {    
    ImageProcess(src, bg);
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

float Tracking::DetectAndTrack(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    int result = 0;
    Mat cur; Mat diff; Mat dst;
    ImageProcess(src, cur);
    subtract(bg, cur, diff);
    float diff_val = sum(diff)[0]/(scale_w * scale_h);
    imwrite("bg.png", bg);
    imwrite("cur.png", cur);
    imwrite("diff.png", diff);

    if(index > 1) {
        Mat same;        
        subtract(prev, cur, same);
        float same_check = sum(same)[0];
        dl.Logger("same check %f ", same_check);
        if (same_check < 10) {
            dl.Logger("Current image is same as previous.. ");
            issame = true;
        }
    }
    cur.copyTo(prev);

    if( index == 1 ) 
        first_summ = diff_val;

    if(isfound == true) {
        Mat mask = Mat(Size(scale_w, scale_h), CV_8UC1);
        rectangle(mask, Point(obj->sx, obj->sy), Point(obj->ex, obj->ey), Scalar(255), -1);
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
    // printf(" OK --- ? \n ");
    // for(int i = 0 ; i < t.size(); i++) 
    //     printf("rect t[i] %d %d %d %d\n", t[i].x, t[i].y, t[i].width, t[i].height);

    // std::sort(t.begin(), t.end(), [](Rect a, Rect b) {
    //          return a.width * a.height <= b.width * b.height;;
    // });
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
        dl.Logger("Object found. obj %d %d %d %d", obj->sx, obj->sy, obj->w, obj->h);
    }
    else if (res_rect.size() > 0 && isfound == true && obj != NULL) {
        bool newfound = false;
        for(int i = 0 ; i < res_rect.size(); i ++){
            float tcen_x = res_rect[i].x + res_rect[i].width/2;
            float tcen_y = res_rect[i].y + res_rect[i].height/2;
            if (abs(tcen_x - obj->cx) < p->center_threshold && abs(tcen_y - obj->cy) < p->center_threshold) {
                obj->id = i;
                obj->update(res_rect[i].x, res_rect[i].y, res_rect[i].width, res_rect[i].height);
                newfound = true;
                break;
            }
        }
        if(newfound == true){
            isfound = true;
            p->keep_tracking = true;
            dl.Logger("keep tracking.. %d %d %d %d", obj->sx, obj->sy, obj->w, obj->h);
        } else {
            isfound = false;
            p->keep_tracking = false;
            dl.Logger("missed !!");
        }
    }

    if (isfound == true) {        
        MakeROI(obj, roi);
    }
    dl.Logger("isfound %d issmae %d diff_val %f ", isfound, issame, diff_val);
    q.clear();
    t.clear();
    return diff_val;
}
    
void Tracking::MakeROI(TRACK_OBJ* obj, TRACK_OBJ* roi) {
    int sx = 0;
    int sy = 0;
    int real_w = max(p->roi_w, obj->w);
    int real_h = max(p->roi_h, obj->h);

    if ((obj->cx + real_w /2) > p->limit_bx)
        real_w = p->limit_bx - obj->sx;
    if ((obj->cy + real_h /2) > p->limit_by)
        real_h = p->limit_by - obj->sy;

    if ((obj->cx - real_w /2) > p->limit_lx)
        roi->sx = obj->cx - (real_w /2);
    else 
        roi->sx = p->limit_lx;
    if ((obj->cy - real_h /2) > p->limit_ly)
        roi->sy = obj->cx - (real_h / 2);
    else
        roi->sy = p->limit_ly;
    roi->w = real_w;
    roi->h = real_h;;
    roi->update();

    dl.Logger("MakeROI %d %d %d %d ", int(roi->sx), int(roi->sy), int(roi->w), int(roi->h));

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
    src.copyTo(temp);

    if(p->track_scale != 1 ) {
        scale_w = int(src.cols/p->track_scale);
        scale_h = int(src.rows/p->track_scale);
        resize(temp, temp, Size(scale_w, scale_h));
        imwrite("check2.png", temp);        
    }
    GaussianBlur(temp, dst, {3, 3}, 0.7, 0.7);
}

void Tracking::DrawObjectTracking(Mat& src, TRACK_OBJ* obj, TRACK_OBJ* roi, bool borigin) {
    if(borigin == false) {
        rectangle(src, Point(obj->sx, obj->sy), Point(obj->ex, obj->ey), (0), 2);
        putText(src, "FOCUS", Point(obj->sx, obj->sy - 10), FONT_HERSHEY_SIMPLEX, 0.4, (0), 1);
        rectangle(src, Point(roi->sx, roi->sy), Point(roi->ex, roi->ey), (0), 2);    
    } else if(borigin == true){
        rectangle(src, Point(obj->sx * p->track_scale, obj->sy * p->track_scale), Point(obj->ex * p->track_scale, obj->ey * p->track_scale), (0), 2);
        putText(src, "FOCUS", Point(obj->sx * p->track_scale, (obj->sy - 20) * p->track_scale), FONT_HERSHEY_SIMPLEX, 0.7, (255), 1);
        rectangle(src, Point(roi->sx* p->track_scale, roi->sy * p->track_scale), Point(roi->ex * p->track_scale, roi->ey * p->track_scale), (0), 2);    
    }
    string contents;
    switch(p->replay_style) {
        case KEEP_TRACKING:
            contents = "Keep tracking";
        case SWIPE_START :
            contents = "Swipe Start";
        case SWIPE_END :
            contents = "Swipe End";
        case MISSED_TRACKING :
            contents = "Missed object";        
        case PAUSE_PERIOD :
            contents = "Pause period ";                
    }

    putText(src, contents, Point( 20, 20), FONT_HERSHEY_SIMPLEX, 1, (0), 1);
}