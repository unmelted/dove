/*****************************************************************************
*                                                                            *
*                           GrayTracking.hpp     							 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : GrayTracking.hpp
    Author(S)       : Me Eunkyung
    Created         : 28 dec 2021

    Description     : GrayTracking.hpp
    Notes           : Tracking
*/
#pragma once
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include "DefData.hpp"
#include "Tracking.hpp"

using namespace std;
using namespace cv;
using namespace dove;

class GrayTracking : public Tracking {
    public:
    GrayTracking();
    ~GrayTracking();

    void SetBg(cuda::GpuMat& src, int frame_id);
    void SetBg(Mat& src, int frame_id);

    int TrackerInit(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);    
    int TrackerUpdate(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);    
    void ImageProcess(Mat& src, Mat& dst);    
};