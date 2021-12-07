
/*****************************************************************************
*                                                                            *
*                           Stabilization.cpp  								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Stabilization.cpp
    Author(S)       : Me Eunkyung
    Created         : 07 dec 2021

    Description     : Stabilization.cpp
    Notes           : Stabilization main class
*/


#include "Stabilization.hpp"

using namespace std;
using namespace cv;

Dove::Dove() {
    p = new PARAM;
    t = new TIMER;
    dl = Dlog();        

}

Dove::Dove(int mode, bool has_mask, int* coord, string id) {
    Dove();
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
        p->sx = coord[0];
        p->sy = coord[1];
        p->width = coord[2];
        p->height = coord[3];    
    }

    p->blur_size = 11;
    p->blur_sigma = 1.3;
    p->dst_width = 1920;
    p->dst_height = 1080;

    smth.create(2 , 3 , CV_64F);        

}

int Dove::ImageProcess(Mat& src, Mat& dst){
    Mat temp;
    if(p->scale != 1)
        resize(src, src, Size(int((float)src.cols/p->scale), int(float(src.rows)/p->scale)), 0,0,1);

    cvtColor(src, temp, COLOR_BGR2GRAY);
    GaussianBlur(temp, dst, {p->blur_size, p->blur_size}, p->blur_sigma, p->blur_sigma);

}

int Dove::CalculateMove(Mat& cur) {
    vector <Point2f> features1, features2;
    vector <Point2f> goodFeatures1, goodFeatures2;
    vector <uchar> status;
    vector <float> err;

    goodFeaturesToTrack(cur, features1, 30, 0.01  , 30, mask, 11, false, 0.04);
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
        //affine = estimateAffine2D(goodFeatures1, goodFeatures2);
        affine = estimateRigidTransform(goodFeatures1, goodFeatures2, false);                
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
}

int Dove::ApplyImage(Mat& src, bool scaled) {
    if(scaled == true) {
        smth.at<double>(0,2) = smth.at<double>(0,2) * p->scale;
        smth.at<double>(1,2) = smth.at<double>(1,2) * p->scale;      
    }
    warpAffine(src1, src1, smth, src1.size());    
}

int Dove::MakeMask(Mat& mask, PARAM* p) {
    cout<< "mask width , height : " << p->dst_width << " , "<< p->dst_height<<endl;
    mask = Mat::zeros(p->dst_height, p->dst_width, CV_8UC1);
    rectangle(mask, Point(p->sx, p->sy), Point(p->sx + p->width, p->sy + p->height), Scalar(255), -1);
    imwrite("mask.png", mask);

    return 1;
}
