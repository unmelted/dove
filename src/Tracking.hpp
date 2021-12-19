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

using namespace std;
using namespace cv;

class Tracking {

    public :
    PARAM* p;
    Dlog dl;
    float first_summ;
    bool isfound;
    bool issame;


    Tracking();
    ~Tracking();
    void SetParam(PARAM* _p) { memcpy(p, _p, sizeof(PARAM));};
    void SetLogFilename(string name) {dl.SetLogFilename(name); };    
    float DetectAndTrack(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);
    void SetBg(Mat& src);
    void DrawObjectTracking(Mat& src, TRACK_OBJ* obj, TRACK_OBJ* roi, bool borigin = false, int replay_stype = 0);

    private :
    char filename[50];
    Mat bg;
    Mat prev;
    Mat diff;
    Ptr<MSER>ms;

    int scale_w;
    int scale_h;
    int prev_summ;
    int prev_diff;

    void MakeROI(TRACK_OBJ* obj, TRACK_OBJ* roi);
    float GetIOU(Rect& r, int index, vector<Rect>& rects);
    bool CheckWithin(Rect& r);
    bool CheckWithin(Rect& r, int index, vector<Rect>& rects);
    void ImageProcess(Mat& src, Mat& dst);
};