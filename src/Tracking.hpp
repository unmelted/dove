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

    Tracking();
    ~Tracking();
    void SetParam(PARAM* _p) { p = _p; };
    int DetectAndTrack(Mat& src, int index, TRACK_OBJ* dobj = NULL);
    int CheckMovieSequence();
    void SetBg(Mat& src);

    private :
    Mat bg;
    Ptr<MSER>ms;
    bool isFound;

    int first_summ;
    int prev_summ;
    int prev_diff;

    void MakeROI();    
    float GetIOU();
    bool CheckWithin(Rect& r);
    bool CheckWithin(Rect& r, int index, vector<Rect>& rects);
    void ImageProcess(Mat& src, Mat& dst);

};