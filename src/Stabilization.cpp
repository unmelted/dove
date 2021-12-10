
/*****************************************************************************
*                                                                            *
*                           Stabilization.cpp  								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Stabilization.cpp
    Author(S)       : Me Eunkyung
    Created         : 07 dec 2021

    Description     : main procee for Stabilization
    Notes           : Stabilization main class
*/


#include "Stabilization.hpp"

using namespace std;
using namespace cv;

Dove::Dove() {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
    dl.SetLogFilename("TEST");
}

Dove::Dove(int mode, bool has_mask, int* coord, string id) {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
    dl.SetLogFilename("TEST");

    dl.Logger("Start construct. %d %d  ", coord[0], coord[1]);
    p->mode = mode;
    if(has_mask == true) 
        Initialize(true, coord);
    else 
        Initialize(false, 0);    
    dl.SetLogFilename(id);        
}

Dove::~Dove() {
    delete p;
    delete t;
}

int Dove::Process() {
};


void Dove::Initialize(bool has_mask, int* coord) {
    p->scale = 2;
    if(has_mask == true) {
        p->has_mask = true;
        p->sx = coord[0] / p->scale;
        p->sy = coord[1] / p->scale;
        p->width = coord[2] / p->scale;
        p->height = coord[3] / p->scale;    
    }

    p->blur_size = 5;
    p->blur_sigma = 0.7;
    p->dst_width = 1920;
    p->dst_height = 1080;

    if(p->run_detection == true) {
        //ntw = load_network();
    }

    smth.create(2 , 3 , CV_64F);        
    dl.Logger("Initialized compelete.");
}

int Dove::ImageProcess(Mat& src, Mat& dst){
    Mat temp;
    if(p->scale != 1)
        resize(src, temp, Size(int((float)src.cols/p->scale), int(float(src.rows)/p->scale)), 0,0,1);

    cvtColor(temp, temp, COLOR_BGR2GRAY);
    GaussianBlur(temp, dst, {p->blur_size, p->blur_size}, p->blur_sigma, p->blur_sigma);

    if(p->has_mask)
        MakeMask();
}
int Dove::CalculateMove(Mat& cur) {
    int result = -1;
    if(p->mode == OPTICALFLOW_LK_2DOF) {
        result = CalculateMove_LK2D(cur);
    } else if (p->mode == OPTICALFLOW_LK_6DOF) {
        result = CalculateMove_LK6D(cur);
    } else if (p->mode == INTEGRAL_IMAGE) {
        result = CalculateMove_Integral(cur);
    } else {
        result = CalculateMove_Tracker(cur);
    }
    return result;
}

int Dove::CalculateMove_LK2D(Mat& cur) {
    //dl.Logger("calculation start..");
    static int i = 1;
    // sprintf(filename, "saved/%d_cur.png", i);
    // imwrite(filename, cur);
    // sprintf(filename, "saved/%d_ref.png", i);
    // imwrite(filename, ref);
    i++;
    vector <Point2f> features1, features2;
    vector <Point2f> goodFeatures1, goodFeatures2;
    vector <uchar> status;
    vector <float> err;

    if(p->has_mask == true)
        goodFeaturesToTrack(cur, features1, 30, 0.01  , 30, mask, 11, false, 0.04);
    else 
        goodFeaturesToTrack(cur, features1, 30, 0.01  , 30, noArray(), 11, false, 0.04);    
    calcOpticalFlowPyrLK(cur, ref, features1, features2, status, err );

    for(size_t i=0; i < status.size(); i++)
    {
        if(status[i])
        {
            goodFeatures1.push_back(features1[i]);
            goodFeatures2.push_back(features2[i]);
        }
    }

    if(goodFeatures1.size() < threshold || goodFeatures2.size() < threshold) {
            cout<< i << " no feature to track.. feature cnt : "<< goodFeatures1.size() << endl;
            pre_affine.copyTo(affine);
    }
    else {
        affine = estimateAffine2D(goodFeatures1, goodFeatures2);
        //affine = estimateRigidTransform(goodFeatures1, goodFeatures2, false);                
    }

    if(affine.empty() == true) {
        dl.Logger("there is no solution ..");
        return -1;
    }

    double dx = affine.at<double>(0,2);
    double dy = affine.at<double>(1,2);
    double da = atan2(affine.at<double>(1,0), affine.at<double>(0,0));
    double ds_x = affine.at<double>(0,0)/cos(da);
    double ds_y = affine.at<double>(1,1)/cos(da);

    smth.at<double>(0,0) = 1; //ds_x * cos(da);
    smth.at<double>(0,1) = 0; //ds_x * -sin(da);
    smth.at<double>(1,0) = 0; //ds_y * sin(da);
    smth.at<double>(1,1) = 1; //ds_y * cos(da);
    smth.at<double>(0,2) = dx;
    smth.at<double>(1,2) = dy;

    dl.Logger("calculate done dx %f dy %f", dx, dy);
    return ERR_NONE;
}

int Dove::CalculateMove_LK6D(Mat& cur) {
    return ERR_NONE;
}

int Dove::CalculateMove_Integral(Mat& cur) {
    return ERR_NONE;
}

int Dove::CalculateMove_Tracker(Mat& cur) {
    return ERR_NONE;
}

int Dove::ApplyImage(Mat& src, bool scaled) {
    if( smth.at<double>(0,2) == 0.0 && smth.at<double>(1,2) == 0.0) {
        dl.Logger("no warp");
        return -1;
    }

    if(scaled == true) {
        smth.at<double>(0,2) = smth.at<double>(0,2) * p->scale;
        smth.at<double>(1,2) = smth.at<double>(1,2) * p->scale;      
    }

    warpAffine(src, src, smth, src.size());
}

int Dove::Detection(Mat& cur) {

}

int Dove::MakeMask() {
    mask = Mat::zeros(p->dst_height, p->dst_width, CV_8UC1);
    rectangle(mask, Point(p->sx, p->sy), Point(p->sx + p->width, p->sy + p->height), Scalar(255), -1);
    imwrite("saved/mask.png", mask);

    return ERR_NONE;
}
