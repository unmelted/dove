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

int Tracking::DetectAndTrack(Mat& src, int index, TRACK_OBJ* dobj) {
    int result = 0;
    Mat cur; Mat diff; Mat dst;
    ImageProcess(src, cur);
    subtract(bg, cur, diff);
    int diff_val = sum(diff)[0];

    if( index == 1 ) 
        first_summ = diff_val;

    if(isFound && dobj != NULL) {
        Mat mask = Mat(Size(640, 480), CV_8UC1);
        rectangle(mask, Point(dobj->sx, dobj->sy), Point(dobj->ex, dobj->ey), Scalar(255), -1);
        bitwise_and(mask, diff, dst);
    }
    else 
        dst = diff;

    vector<vector<Point>>res_pt;
    vector<Rect>res_rect;
    vector<Rect>t;    
    ms->detectRegions(dst, res_pt, res_rect);

    for(int i = 0; i < res_rect.size() ; i ++ ) {
        if(CheckWithin(res_rect[i]) == true)
            t.push_back(res_rect[i]);
    }
    sort(t.begin(), t.end(), Compare);

    prev_diff = diff_val;

    return ERR_NONE;
}

int Tracking::CheckMovieSequence() {

    return ERR_NONE;
}
    
void Tracking::MakeROI() {

}

float Tracking::GetIOU() {

    return ERR_NONE;
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
        resize(src, src, Size(int(src.cols/p->track_scale), int(src.rows/p->track_scale)), 0, 0, 1);
    }
    cvtColor(src, src, COLOR_BGR2GRAY);
    GaussianBlur(src, dst, {3, 3}, 0.7, 0.7);
}
