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


Tracking::Tracking() {
    ms = MSER::create();
}

Tracking::~Tracking() {

}

void Tracking::SetBg(Mat& src) {    
    ImageProcess(src, bg);
}

bool Compare(Rect a, Rect b) {
    return a.width * a.height <= b.width * b.height;
}

float Tracking::DetectAndTrack(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    int result = 0;
    Mat cur; Mat diff; Mat dst;
    ImageProcess(src, cur);
    subtract(bg, cur, diff);
    float diff_val = sum(diff)[0]/(scale_w * scale_h);

    if( index == 1 ) 
        first_summ = diff_val;

    if(isfound && obj != NULL) {
        Mat mask = Mat(Size(640, 480), CV_8UC1);
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

    for(int i = 0; i < res_rect.size() ; i ++ ) {
        if(CheckWithin(res_rect[i]) == true)
            t.push_back(res_rect[i]);
    }
    sort(t.begin(), t.end(), Compare);
    for(int i = 0 ; i < t.size(); i++){
        if(CheckWithin(t[i], i, t) == false)
            q.push_back(t[i]);
    }
    res_rect.clear();    
    for (int i = 0 ; i < q.size(); i ++) {
        if(GetIOU(q[i], i, q) < p->iou_threshold) {
            res_rect.push_back(q[i]);
        }
    }
    int last = res_rect.size() - 1;
    if (res_rect.size() > 0 && isfound == false && res_rect[last].width * res_rect[last].height > p->area_threshold) {
        isfound = true;
        TRACK_OBJ tobj(res_rect[last].x, res_rect[last].y, res_rect[last].width, res_rect[last].height);
        tobj.id = last;
        tobj.update();
        obj = &tobj;

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
            dl.Logger("keep tracking.. ");
        } else {
            isfound = false;
            p->keep_tracking = false;
            dl.Logger("missed !!");
        }
    }

    if (isfound == true) {        
        TRACK_OBJ troi;
        MakeROI(obj, &troi);
    }

    return diff_val;
}
    
void Tracking::MakeROI(TRACK_OBJ* obj, TRACK_OBJ* roi) {
    int sx = 0;
    int sy = 0;
    int real_w = max(p->roi_w, obj->w);
    int real_h = max(p->roi_h, obj->h);

    if ((obj->cx + real_w /2) > p->limit_bx)
        real_w = real_w - ((obj->cx + real_w/2) - p->limit_bx);
    if ((obj->cy + real_h /2) > p->limit_by)
        real_h = real_h - ((obj->cy + real_h / 2) - p->limit_by);

    if ((obj->cx - real_w /2) > p->limit_lx)
        roi->sx = obj->cx - (real_w /2);
    else 
        roi->sx = p->limit_lx;
    if ((obj->cy - real_h /2) > p->limit_ly)
        roi->sy = obj->cx - (real_h / 2);
    else
        roi->sy = p->limit_ly;

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
        int inter_y = min(r.y + r.height, rects[i].y + rects[i].height) - max(r.y, rects[i].y);

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
        if( r.x >= rects[i].x && r.y >= rects[i].y &&
            r.width <= rects[i].width && r.height <= rects[i].height)
                return true;
    }
    return false;
}

void Tracking::ImageProcess(Mat& src, Mat& dst) {

    if(p->track_scale == 1 ) {
        scale_w = int(src.cols/p->track_scale);
        scale_h = int(src.rows/p->track_scale);
        resize(src, src, Size(scale_w, scale_h), 0, 0, 1);
    }
    cvtColor(src, src, COLOR_BGR2GRAY);
    GaussianBlur(src, dst, {3, 3}, 0.7, 0.7);
}
