/*****************************************************************************
*                                                                            *
*                           GrayTracking.cpp     							 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : GrayTracking.cpp
    Author(S)       : Me Eunkyung
    Created         : 28 dec 2021

    Description     : GrayTracking.cpp
    Notes           : Tracking
*/
#include "GrayTracking.hpp"
#include <opencv2/core/cuda.hpp>

GrayTracking::GrayTracking() {

}

GrayTracking::~GrayTracking() {

}

void GrayTracking::SetBg(Mat& src, int frame_id) {    
    int histbin = 256;
    double minval; double maxval;
    double cut_threshold;
    Point minloc; Point maxloc;
    Mat hist;
    start_frame = frame_id;

    if(p->track_scale != 1 ) {
        scale_w = int(src.cols/p->track_scale);
        scale_h = int(src.rows/p->track_scale);
        cv::resize(src, bg, Size(scale_w, scale_h));
    }    
    calcHist(&bg, 1, 0, Mat(), hist, 1, &histbin, 0);
    // for (int i = 0 ; i < histbin; i ++){
    //     printf(" [%d] hist %d \n", i , (int)hist.at<float>(i));
    // }

    cv::minMaxLoc(hist, &minval, &maxval, &minloc, &maxloc, Mat());
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

    Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(20);
    clahe->apply(result, temp);
    GaussianBlur(temp, bg, {3, 3}, 1.3, 1.3);
    dl.Logger("Setbg function finish %d %d ", bg.cols, bg.rows);
}

void GrayTracking::ImageProcess(Mat& src, Mat& dst) {
    Mat temp;
    Mat result;
    src.copyTo(temp);

    if(p->track_scale != 1 ) {
        scale_w = int(src.cols/p->track_scale);
        scale_h = int(src.rows/p->track_scale);
        cv::resize(temp, temp, Size(scale_w, scale_h));
//        imwrite("check2.png", dst);
    }
    LUT(temp, lut, result);

    Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(20);
    clahe->apply(result, temp);
    GaussianBlur(temp, dst, {5, 5}, 1.3, 1.3);
}

#if defined GPU
void GrayTracking::SetBg(cuda::GpuMat& src, int frame_id) {
    int histbin = 256;
    double minval; double maxval;
    double cut_threshold;
    Point minloc; Point maxloc;
    Mat hist;// = Mat::zeros(Size(256, 1), CV_32SC1);
    start_frame = frame_id;

    if(p->track_scale != 1 ) {
        scale_w = int(src.cols/p->track_scale);
        scale_h = int(src.rows/p->track_scale);
        cuda::resize(src, bgg, Size(scale_w, scale_h));
    }    
    cuda::calcHist(bg, hist, cuda::Stream::Null());
    // for (int i = 0 ; i < histbin; i ++){
    //     printf(" [%d] hist %d \n", i , (int)hist.at<float>(i));
    // }

    cuda::minMaxLoc(hist, &minval, &maxval, &minloc, &maxloc, noArray());
    printf("searched minval %f maxval %f minloc %d %d maxloc %d %d", minval, maxval, minloc.x, minloc.y, maxloc.x, maxloc.y);
    cut_threshold = maxloc.y * 0.83;
    lut = Mat::zeros(1, histbin, CV_8UC1);
    for (int i = 0 ; i < histbin; i ++){
        if(i <= cut_threshold) 
            lut.at<unsigned char>(i) = i;
        else 
            lut.at<unsigned char>(i) = 255;
        printf(" [%d] lut--  %d \n", i , lut.at<unsigned char>(i));            
    }

    cuda::GpuMat gt; 
    gt.upload(lut);
    cuda::GpuMat result; 
    cuda::GpuMat temp;
    Ptr<cuda::LookUpTable> glut = cuda::createLookUpTable(lut);
    glut->transform(bgg, result);

    Ptr<cuda::CLAHE> clahe = cuda::createCLAHE(20);
    clahe->apply(result, temp);
    Ptr<cuda::Filter>gblur = cuda::createGaussianFilter(CV_8UC1, CV_8UC1, Size(3, 3), 1.3);
    gblur->apply(temp, bgg);
    gt.release();
    result.release();
    temp.release();

    Mat check;
    bgg.download(check);
    imwrite("gpu_bg.png", check);
    dl.Logger("Setbg function finish %d %d ", bg.cols, bg.rows);
}

void GrayTracking::ImageProcess(cuda::GpuMat& src, cuda::GpuMat& dst) {
    cuda::GpuMat temp;
    cuda::GpuMat result;
    src.copyTo(temp);    
    if(p->track_scale != 1 ) {
        scale_w = int(src.cols/p->track_scale);
        scale_h = int(src.rows/p->track_scale);
        cuda::resize(temp, dst, Size(scale_w, scale_h));
//        imwrite("check2.png", dst);
    }

}

int GrayTracking::TrackerInit(cuda::GpuMat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    int result = 0;
    double minval; double maxval;
    Point minloc; Point maxloc;
    cuda::GpuMat cur;
    ImageProcess(src, cur);
    dl.Logger("PickArea cos/row %d %d st_frame %d index %d", cur.cols, cur.rows, start_frame, index);
    cuda::subtract(bgg, cur, diffg);
    float diff_val = cuda::sum(diffg)[0] / (scale_w * scale_h);

    cuda::minMaxLoc(diffg, &minval, &maxval, &minloc, &maxloc, Mat());
    dl.Logger("PickArea minval %f maxval %f minloc %d %d maxloc %d %d", minval, maxval, minloc.x, minloc.y, maxloc.x, maxloc.y);
    diffg.download(diff);

    obj->update(maxloc.x - 30, maxloc.y - 30, 60, 90);
    obj->update();
    roi->update(obj->sx - 10, obj->sy - 10, obj->w + 20, obj->h + 20);
    roi->update();
    dl.Logger("gray obj %d %d %d %d", obj->sx, obj->sy, obj->w, obj->h);
    dl.Logger("gray roi %d %d %d %d", roi->sx, roi->sy, roi->w, roi->h);
    ConvertToRect(roi, &rect_roi);
    dl.Logger("gray rect roi for tracker init %d %d %d %d", rect_roi.x, rect_roi.y, rect_roi.width, rect_roi.height);
    tracker->init(diff, rect_roi);
    isfound = true;
    //DrawObjectTracking(diff, obj, roi, false, 1);
    return ERR_NONE;
}

int GrayTracking::TrackerUpdate(cuda::GpuMat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    cuda::GpuMat cur;
    ImageProcess(src, cur);
    //dl.Logger("TrackerUpdate cos/row %d %d st_frame %d index %d", cur.cols, cur.rows, start_frame, index);
    cuda::subtract(bgg, cur, diffg);
    float diff_val = cuda::sum(diff)[0] / (scale_w * scale_h);
    /* if you need to check the same image, please uncommnet these block.
    if(index > start_frame +1 && !prev.empty()) {
        Mat same;
        subtract(prev, cur, same);
        float same_check = sum(same)[0]/(scale_w * scale_h);
        //dl.Logger("same check %f ", same_check);
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
    cur.copyTo(prevg);
    */


    diffg.download(diff);
    bool ret = tracker->update(diff, rect_roi);
    dl.Logger("[%d] tracker update %d %d %d %d ", index, rect_roi.x, rect_roi.y, rect_roi.width, rect_roi.height);

    if (ret == false) {
        dl.Logger("tracker miss --------------------------------------------");
        //        tracker->init(diff, rect_roi);            
    }

    ConvertToROI(rect_roi, obj, roi);
    isfound = true;
    //DrawObjectTracking(diff, obj, roi, false, 1);
    //sprintf(filename, "saved\\%d_trck.png", index);
    //imwrite(filename, diff);
    tracker->init(diff, rect_roi);
    if (p->mode == DETECT_TRACKING_CH) {
        MakeROI(obj, feature_roi);
        ConvertToRect(feature_roi, &rect_feature_roi, p->track_scale);
    }

    return ERR_NONE;
}

#endif

int GrayTracking::TrackerInit(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    int result = 0;
    double minval; double maxval;
    Point minloc; Point maxloc;
    Mat cur; Mat dst;
    ImageProcess(src, cur);
    dl.Logger("PickArea cos/row %d %d st_frame %d index %d", cur.cols, cur.rows, start_frame, index);
    cv::subtract(bg, cur, diff);
    float diff_val = cv::sum(diff)[0]/(scale_w * scale_h);

    cv::minMaxLoc(diff, &minval, &maxval, &minloc, &maxloc, Mat());
    dl.Logger("PickArea minval %f maxval %f minloc %d %d maxloc %d %d", minval, maxval, minloc.x, minloc.y, maxloc.x, maxloc.y);

    obj->update(maxloc.x -30, maxloc.y -30, 60, 90);
    obj->update();
    roi->update(obj->sx - 10, obj->sy - 10, obj->w + 20, obj->h + 20);    
    roi->update();
    dl.Logger("gray obj %d %d %d %d", obj->sx, obj->sy ,obj->w , obj->h);
    dl.Logger("gray roi %d %d %d %d", roi->sx, roi->sy ,roi->w , roi->h);
    ConvertToRect(roi, &rect_roi);
    dl.Logger("gray rect roi for tracker init %d %d %d %d", rect_roi.x, rect_roi.y, rect_roi.width, rect_roi.height);
    tracker->init(diff, rect_roi);
    isfound = true;
    //DrawObjectTracking(diff, obj, roi, false, 1);
    return ERR_NONE;
}

int GrayTracking::TrackerUpdate(Mat& src, int index, TRACK_OBJ* obj, TRACK_OBJ* roi) {
    Mat cur; Mat dst;
    ImageProcess(src, cur);
    //dl.Logger("TrackerUpdate cos/row %d %d st_frame %d index %d", cur.cols, cur.rows, start_frame, index);
    cv::subtract(bg, cur, diff);
    float diff_val = cv::sum(diff)[0]/(scale_w * scale_h);
    /* if you need to check the same image, please uncommnet these block.
    if(index > start_frame +1 && !prev.empty()) {
        Mat same;        
        subtract(prev, cur, same);
        float same_check = sum(same)[0]/(scale_w * scale_h);
        //dl.Logger("same check %f ", same_check);
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
    */
  
    bool ret = tracker->update(diff, rect_roi);
    dl.Logger("[%d] tracker update %d %d %d %d ",index, rect_roi.x, rect_roi.y, rect_roi.width, rect_roi.height);
   
    if (ret == false) {
        dl.Logger("tracker miss --------------------------------------------");
//        tracker->init(diff, rect_roi);            
    }

    ConvertToROI(rect_roi, obj, roi);
    isfound = true;    
    //DrawObjectTracking(diff, obj, roi, false, 1);
    //sprintf(filename, "saved\\%d_trck.png", index);
    //imwrite(filename, diff);
    tracker->init(diff, rect_roi);                    
    if(p->mode == DETECT_TRACKING_CH) {
        MakeROI(obj, feature_roi);
        ConvertToRect(feature_roi, &rect_feature_roi, p->track_scale);
    }

    return ERR_NONE;
}
