/*****************************************************************************
*                                                                            *
*                           Tracking.hpp     								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Tracking.hpp
    Author(S)       : Me Eunkyung
    Created         : 17 dec 2021

    Description     : Tracking.hpp
    Notes           : Tracking
*/

#include "DefData.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

using namespace std;
using namespace cv;
using namespace dove;

class Tracking {

    public :      
    PARAM* p;                                     
    Dlog dl;
    int start_frame;
    float first_summ;
    bool isfound;
    bool issame;
    Rect rect_feature_roi;

    Tracking();
    ~Tracking();
    void SetInitialData(PARAM* _p);
    void SetLogFilename(string name) {this->dl.SetLogFilename(name); };    
    float DetectAndTrack(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);
    int TrackerUpdate(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);
    void SetBg(Mat& src, int frame_id);
    int PickArea(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);    
    void DrawObjectTracking(Mat& src, TRACK_OBJ* obj, TRACK_OBJ* roi, bool borigin = false, int replay_stype = 0);
    void DrawObjectTracking(TRACK_OBJ* obj, TRACK_OBJ* roi, vector<Rect> rects);
    int PickAreaFx(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);
    int TrackerUpdateFx(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);

    private :
    char filename[50];
    Mat bg;
    Mat prev;
    Mat diff;
    Ptr<MSER>ms;
    Ptr<Tracker>tracker;
    Rect rect_roi;
    TRACK_OBJ* feature_roi;


    Mat lut;
    int scale_w;
    int scale_h;
    int prev_summ;
    int prev_diff;

    void MakeROI(TRACK_OBJ* obj, TRACK_OBJ* roi);
    float GetIOU(Rect& r, int index, vector<Rect>& rects);
    bool CheckWithin(Rect& r);
    bool CheckWithin(Rect& r, int index, vector<Rect>& rects);
    void ImageProcess(Mat& src, Mat& dst);
    void ConvertToRect(TRACK_OBJ* roi, Rect* rec, int scale = 1);
    void ConvertToROI(Rect& rec, TRACK_OBJ* obj, TRACK_OBJ* roi);
};