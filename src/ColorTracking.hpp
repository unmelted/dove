/*****************************************************************************
*                                                                            *
*                           ColorTracking.hpp     							 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : ColorTracking.hpp
    Author(S)       : Me Eunkyung
    Created         : 28 dec 2021

    Description     : ColorTracking.hpp
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


class ColoredTracking : public Tracking {
    public:
    ColoredTracking();
    ~ColoredTracking();

    void SetBg(Mat& src, int frame_id);
    void ImageProcess(Mat& src, Mat& dst);
    int TrackerInit(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);    
    int TrackerUpdate(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);    
#if defined GPU    
    void SetBg(cuda::GpuMat& src, int frame_id);    
    void ImageProcess(GpuMat& src, Mat& dst);    
#endif    

};
