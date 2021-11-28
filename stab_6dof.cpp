
/*****************************************************************************
*                                                                            *
*                            stab_6dof         								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : stab_6dof.cpp
    Author(S)       : Me Eunkyung
    Created         : 28 nov 2021

    Description     : stab_6dof.cpp
    Notes           : 6dof video stabilization with opticalflow
*/
#pragma once 
#include "stab.hpp"

int stab_6dof(char* in, char* out) {

    VideoCapture stab(in);
    VideoWriter output;
    //VideoWriter dual;    

    Mat src1; Mat src1oc; Mat src1o;
    Mat mask;
    Mat src2;
    Mat smth;

    Mat pre_affine;
    Mat affine;
    int cp_width = 0;
    int cp_height = 0;
    Rect srcrect;
    Rect dstrect;
    char filename[30];
    int scale = 4;
    int i = 0;
    int threshold = 6;

    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    
    smth.create(2 , 3 , CV_64F);    
    MakeMask6(mask, 1920/scale, 1080/scale);

    output.open(out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(1920, 1080));

    while(true) {

        stab >> src1oc;
        if(src1oc.data == NULL)
            break;

        resize(src1oc, src1o, Size(int((float)src1oc.cols/scale), int(float(src1oc.rows)/scale)), 0,0,1);
        cvtColor(src1o, src1o, COLOR_BGR2GRAY);

        if( i == 0) {
            src2 = src1o;
            sprintf(filename, "saved/%d_src1.png", i);
            imwrite(filename, src1oc);
            i++;
            continue;
        }
        /*
        if(cp_width == 0 ) {
            src1o.copyTo(src1);
        }
        else if(owidth != cp_width) {
            src1 = src1o(dstrect);
        } else {
            src1o.copyTo(src1);            
        }*/

        src1o.copyTo(src1);            
        //cout<< " i : " <<i << " start " << endl; 
        // sprintf(filename, "saved/%d_src1.png", i);        
        // imwrite(filename, src1);
        // sprintf(filename, "saved/%d_src2.png", i);        
        // imwrite(filename, src2);

        vector <Point2f> features1, features2;
        vector <Point2f> goodFeatures1, goodFeatures2;
        vector <uchar> status;
        vector <float> err;

        goodFeaturesToTrack(src1, features1, 30, 0.01  , 30, noArray(), 11, false, 0.04);
        calcOpticalFlowPyrLK(src1, src2, features1, features2, status, err );

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
             //sprintf(filename, "%d_no_feature.png", i);
             //imwrite(filename, src1oc);
             pre_affine.copyTo(affine);
         }
         else {
            affine = estimateAffine2D(goodFeatures1, goodFeatures2);
            //affine = estimateRigidTransform(goodFeatures1, goodFeatures2, false);                
        }

        double dx = affine.at<double>(0,2);
        double dy = affine.at<double>(1,2);
        double da = atan2(affine.at<double>(1,0), affine.at<double>(0,0));
        double ds_x = affine.at<double>(0,0)/cos(da);
        double ds_y = affine.at<double>(1,1)/cos(da);
        double sx = ds_x;
        double sy = ds_y;
        //cout << "dx : " << dx << " dy : "<< dy << " a : " << da << " dsx : " <<ds_x << " ds_y : " <<ds_y << endl;
        //cout << "dx : " << dx * scale<< " dy : "<< dy * scale<< " a : " << da << " dsx : " <<ds_x << " ds_y : " <<ds_y << endl;        

        sum_transX += dx;
        sum_transY += dy;
        sum_thetha += da;
        sum_scaleX += ds_x;
        sum_scaleY += ds_y;

        if(i > 1)
            Kalman_Filter(&scaleX , &scaleY , &thetha , &transX , &transY); 

        diff_scaleX = scaleX - sum_scaleX;
        diff_scaleY = scaleY - sum_scaleY;
        diff_transX = transX - sum_transX;
        diff_transY = transY - sum_transY;
        diff_thetha = thetha - sum_thetha;

        ds_x = ds_x + diff_scaleX;
        ds_y = ds_y + diff_scaleY;
        dx = dx + diff_transX;
        dy = dy + diff_transY;
        da = da + diff_thetha;

        /* nokalman
        smth.at<double>(0,0) = ds_x * cos(da);
        smth.at<double>(0,1) = ds_x * -sin(da);
        smth.at<double>(1,0) = ds_y * sin(da);
        smth.at<double>(1,1) = ds_y * cos(da); */
        smth.at<double>(0,0) = sx * cos(da);
        smth.at<double>(0,1) = sx * -sin(da);
        smth.at<double>(1,0) = sy * sin(da);
        smth.at<double>(1,1) = sy * cos(da);

        smth.at<double>(0,2) = dx;
        smth.at<double>(1,2) = dy;

        warpAffine(src1, src1, smth, src1.size());

        //Mat canvas = Mat::zeros(1080, src1oc.cols*2 +10, src1oc.type());          
        //src1oc.copyTo(canvas(Range::all(), Range(0, src1oc.cols)));
        smth.at<double>(0,2) = -dx * scale;
        smth.at<double>(1,2) = -dy * scale;      

        warpAffine(src1oc, src1oc, smth, src1oc.size());
        sprintf(filename, "saved/%d_src1_warp.png", i);
        imwrite(filename, src1oc);
        i++;
        //src1oc.copyTo(canvas(Range::all(), Range(1930, 3850)));        

        output << src1oc;
        //dual << canvas;
        src1.copyTo(src2);     
        affine.copyTo(pre_affine);   
        Logger("[%d] %f ", i, LapTimer(all));        
    }
}

void Kalman_Filter(double *scaleX , double *scaleY , double *thetha , double *transX , double *transY)
{
    double frame_1_scaleX = *scaleX;
    double frame_1_scaleY = *scaleY;
    double frame_1_thetha = *thetha;
    double frame_1_transX = *transX;
    double frame_1_transY = *transY;

    double frame_1_errscaleX = errscaleX + Q_scaleX;
    double frame_1_errscaleY = errscaleY + Q_scaleY;
    double frame_1_errthetha = errthetha + Q_thetha;
    double frame_1_errtransX = errtransX + Q_transX;
    double frame_1_errtransY = errtransY + Q_transY;

    double gain_scaleX = frame_1_errscaleX / (frame_1_errscaleX + R_scaleX);
    double gain_scaleY = frame_1_errscaleY / (frame_1_errscaleY + R_scaleY);
    double gain_thetha = frame_1_errthetha / (frame_1_errthetha + R_thetha);
    double gain_transX = frame_1_errtransX / (frame_1_errtransX + R_transX);
    double gain_transY = frame_1_errtransY / (frame_1_errtransY + R_transY);

    *scaleX = frame_1_scaleX + gain_scaleX * (sum_scaleX - frame_1_scaleX);
    *scaleY = frame_1_scaleY + gain_scaleY * (sum_scaleY - frame_1_scaleY);
    *thetha = frame_1_thetha + gain_thetha * (sum_thetha - frame_1_thetha);
    *transX = frame_1_transX + gain_transX * (sum_transX - frame_1_transX);
    *transY = frame_1_transY + gain_transY * (sum_transY - frame_1_transY);

    errscaleX = ( 1 - gain_scaleX ) * frame_1_errscaleX;
    errscaleY = ( 1 - gain_scaleY ) * frame_1_errscaleX;
    errthetha = ( 1 - gain_thetha ) * frame_1_errthetha;
    errtransX = ( 1 - gain_transX ) * frame_1_errtransX;
    errtransY = ( 1 - gain_transY ) * frame_1_errtransY;
}


int MakeMask6(Mat& mask, int width, int height) {
    cout<< "mask width , height : " << width << " , "<< height<<endl;
    int border = width/12;
    mask = Mat::zeros(height, width, CV_8UC1);
    rectangle(mask, Point(border, border), Point(width - border, height - border), Scalar(255), -1);
    imwrite("mask.png", mask);
}