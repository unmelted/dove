
/*****************************************************************************
*                                                                            *
*                            stab            								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : stab.hpp
    Author(S)       : Me Eunkyung
    Created         : 28 nov 2021

    Description     : stab.hpp
    Notes           : video stabil header
*/

#pragma once 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/flann/flann.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <fstream>
#include <sys/time.h>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include "common/TimeUtil.hpp"

using namespace std;
using namespace cv;

typedef enum _err {
    ERR_NONE = 0,
    
    EXECUTE_CLIENT_EXCEPTION    = -30,
} ERR;

typedef enum _calmode {

    OPTICALFLOW_LK_2DOF = 1,
    OPTICALFLOW_LK_6DOF = 2,
    INTEGRAL_IMAGE      = 3,
    TRACKER_2DOF        = 4,
    TWOPASS             = 5,
    SIMPLE_KALMAN_LIVE  = 6,
    PATH_SMOOTHE        = 7,

}CALMODE;

typedef enum _masktype { 
    RECT    = 1,
    CIRCLE  = 2,
}MASKTYPE;

typedef struct _param {    
    bool has_mask;
    float scale;
    int mode;
    int blur_size;
    float blur_sigma;

    bool run_kalman;
    bool run_detection;
    
    int sx;
    int sy;
    int width;
    int height;
    int range;
    int stop_threshold;
    int same_threshold;

    int dst_width;
    int dst_height;

}PARAM;

typedef struct _win_info {
    Mat itg;
    int glb_x;
    int glb_y;
    int loc_x;
    int loc_y;
    int range;

    int width;
    int height;
    int srch_x;
    int srch_y;

    int tt_width;
    int tt_height;

    int min_dx;
    int min_dy;

    int min_sum_diff;

    void init() {
        glb_x = 0;
        glb_y = 0;
        width = 0;
        height = 0;            
        loc_x = 0;
        loc_y = 0;
        srch_x = 0;
        srch_y = 0;
        tt_width = 0;
        tt_height =0;

        min_dx = 0;
        min_dy = 0;

        min_sum_diff = 0;
    }

} WIN_INFO;

typedef struct _kalman {

    #define Q1 0.004
    #define R1 0.2
    double sum_scaleX = 0;
    double sum_scaleY = 0;
    double sum_thetha = 0;
    double sum_transX = 0;
    double sum_transY = 0;
    double scaleX = 0;
    double scaleY = 0;
    double thetha = 0;
    double transX = 0;
    double transY = 0;
    double diff_scaleX = 0;
    double diff_scaleY = 0;
    double diff_transX = 0;
    double diff_transY = 0;
    double diff_thetha = 0;
    double errscaleX = 1;
    double errscaleY = 1;
    double errthetha = 1;
    double errtransX = 1;
    double errtransY = 1;
    double Q_scaleX = Q1;
    double Q_scaleY = Q1;
    double Q_thetha = Q1;
    double Q_transX = Q1;
    double Q_transY = Q1;
    double R_scaleX = R1;
    double R_scaleY = R1;
    double R_thetha = R1;
    double R_transX = R1;
    double R_transY = R1;
} KALMAN;

int stab_2dof(char* in, char* out, int coord[4]);
int stab_live(char* infile);
int stab_pathsmoothe(char* infile);
//int test_search();