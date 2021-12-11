
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

typedef enum _detectortype {
    DARKNET_YOLOV4     = 1,
    DARKNET_RTGPU     = 2,
}DT_TYPE;

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
    int detector_type;
    string names_file;
    string cfg_file;
    string weights_file;

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

typedef struct trans
{
    trans() {}
    trans(double _dx, double _dy, double _da) {
        dx = _dx;
        dy = _dy;
        da = _da;
    }

    double dx;
    double dy;
    double da; // angle

} TransformParam;

typedef struct traj
{
    traj() {}
    traj(double _x, double _y, double _a) {
        x = _x;
        y = _y;
        a = _a;
    }

	friend traj operator+(const traj &c1,const traj  &c2){
		return traj(c1.x+c2.x,c1.y+c2.y,c1.a+c2.a);
	}

	friend traj operator-(const traj &c1,const traj  &c2){
		return traj(c1.x-c2.x,c1.y-c2.y,c1.a-c2.a);
	}

	friend traj operator*(const traj &c1,const traj  &c2){
		return traj(c1.x*c2.x,c1.y*c2.y,c1.a*c2.a);
	}

	friend traj operator/(const traj &c1,const traj  &c2){
		return traj(c1.x/c2.x,c1.y/c2.y,c1.a/c2.a);
	}
    void set(double _x, double _y, double _a) {
        x = _x;
        y = _y;
        a = _a;
    }

	traj operator =(const traj &rx){
		x = rx.x;
		y = rx.y;
		a = rx.a;
		return traj(x,y,a);
	}

    double x;
    double y;
    double a; // angle
} Trajectory;

typedef struct _kalman {
    double a = 0;
    double x = 0;
    double y = 0;

	Trajectory X;//posteriori state estimate
	Trajectory	X_;//priori estimate
	Trajectory P;// posteriori estimate error covariance
	Trajectory P_;// priori estimate error covariance
	Trajectory K;//gain
	Trajectory	z;//actual measurement
	double pstd = 4e-3;//can be changed
	double cstd = 0.25;//can be changed
	Trajectory Q;// process noise covariance
	Trajectory R;// measurement noise covariance 

    ofstream out_transform;
    ofstream out_trajectory;
    ofstream out_smoothed;
    ofstream out_new;

} KALMAN;

int stab_2dof(char* in, char* out, int coord[4]);
int stab_live(char* infile);
int stab_pathsmoothe(char* infile);
//int test_search();