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
#include "DefData.hpp"
#include "Tracking.hpp"

using namespace std;
using namespace cv;
using namespace dove;

class GrayTracking : public Tracking {
    public:
    GrayTracking();
    ~GrayTracking();

    void SetBg(Mat& src, int frame_id);
    void ImageProcess(Mat& src, Mat& dst);    
    int TrackerInit(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);    
    int TrackerUpdate(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);    
#if defined GPU
    void SetBg(cuda::GpuMat& src, int frame_id);        
    void ImageProcess(cuda::GpuMat& src, cuda::GpuMat& dst);
    int TrackerInit(cuda::GpuMat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);
    int TrackerUpdate(cuda::GpuMat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi);
#endif
};