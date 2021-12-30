
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
//#include <sys/time.h>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include "common/TimeUtil.hpp"
#if defined _MAC_
#include "darknet/yolo_v2_class.hpp"
#endif

#if defined _WIN_ || _WINDOWS
#define _IMGDEBUG
#define _DEBUG
#define GPU

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#else
//#define _DEBUG
#endif


using namespace std;
using namespace cv;
namespace dove {
typedef enum _err {
    ERR_NONE = 0,
    
    EXECUTE_CLIENT_EXCEPTION    = -30,
} ERR;

typedef enum _event {
    FIGURE          = 1,
    HOCKEY          = 2,
    SHORT           = 3,
}EVENT;

typedef enum _calmode {

    OPTICALFLOW_LK_2DOF     = 1,
    OPTICALFLOW_LK_6DOF     = 2,
    INTEGRAL_IMAGE          = 3,
    TRACKER_2DOF            = 4,
    TWOPASS                 = 5,
    SIMPLE_KALMAN_LIVE      = 6,
    PATH_SMOOTHE            = 7,
    DARKNET_DETECT_MOVE     = 8,
    DETECT_TRACKING         = 9,
    DETECT_TRACKING_CH      = 10,

}CALMODE;

typedef enum _track_return {
    TRACK_NONE          = 0,
    KEEP_TRACKING       = 1,
    SWIPE_ON            = 2,
    KEEP_TRACKING_SWIPE = 3,            
    SWIPE_END           = 4,
    MISSED_TRACKING     = 5,
}TRACK_RESULT;
;
typedef enum _detectortype {
    DARKNET_YOLOV4  = 1,
    DARKNET_RTGPU   = 2,
    BLOB_MSER       = 3,
}DT_TYPE;

typedef enum _trackertype {
    TRACKER_NONE= 0,
    BOOSTING    = 1,
    CSRT        = 2,
    GOTURN      = 3,    
    KCF         = 4,    
    MEDIANFLOW  = 5,
    MIL         = 6,
    MOSSE       = 7,    
    TLD         = 8,
}TRCK_TYPE;

typedef struct _trackobj {
    int id;
    int sx;
    int sy;
    int w;
    int h;    
    float cx;
    float cy;
    int ex;
    int ey;

    _trackobj() {}
    _trackobj(int a, int b, int c, int d) {
        sx = a;
        sy = b;
        w = c;
        h = d;
    }
    void update(int a, int b, int c, int d) {
        sx = a;
        sy = b;
        w = c;
        h = d;
    }
    void update() {
        cx = sx + w/2;
        cy = sy + h/2;
        ex = sx + w;
        ey = sy + h;
    }
    int GetArea() {
        return w * h;
    }
    void copy(_trackobj* b) {
        b->sx = sx;
        b->sy = sy;
        b->w = w;
        b->h = h;
        b->cx = cx;
        b->cy = cy;
        b->ex = ex;
        b->ey = ey;
    }

} TRACK_OBJ;

#if defined _MAC_
typedef struct _dtobjs {
    int frame_id;
    int obj_cnt;
    vector<int>cx;
    vector<int>cy;

    vector<bbox_t>bbx;
    _dtobjs() {
        frame_id = -1;
        obj_cnt = -1;
    }
    _dtobjs(int id, int cnt, vector<bbox_t> b) {
        frame_id = id;
        obj_cnt = cnt;
        bbx.resize(b.size());
        copy(b.begin(), b.end(), bbx.begin());
    }
    _dtobjs(int id) {
        frame_id = id;
        obj_cnt = 0;
    }
    void calCenter() {
        for(int i = 0; i < obj_cnt; i ++){ 
            cx.push_back((bbx[i].x + bbx[i].w) / 2);
            cy.push_back((bbx[i].y + bbx[i].h) / 2);
        }
    }    
}DT_OBJECTS;
#endif

typedef struct _dtxy {
    int dx;
    int dy;
}DT_XY;

typedef enum _masktype { 
    RECT    = 1,
    CIRCLE  = 2,
}MASKTYPE;

typedef struct _param {   
    int event; 
    float scale;
    int mode;
    bool colored; //default value is false. if you want to use colored image, then set true.
    bool roi_input;
    int roi_sx;
    int roi_sy;
    
    int blur_size;
    float blur_sigma;

    bool run_kalman;
    bool run_kalman_pre;
    bool run_kalman_post;
    bool run_path_smoothing;    
    int smoothing_radius;
    //detection
    bool run_detection;
    int detector_type;
    string names_file;
    string cfg_file;
    string weights_file;
    vector<int>id_filter;
    float detect_threshold;

    //tracking
    bool run_tracking;
    int tracker_type;
    int track_scale;
    int limit_lx;
    int limit_ly;
    int limit_bx;
    int limit_by;
    int roi_w;
    int roi_h;
    float area_threshold;
    float iou_threshold;
    float center_threshold;
    bool keep_tracking;    
    float swipe_threshold;

    //genearal
    int swipe_start;
    int swipe_end;    

    int sx;
    int sy;
    int width;
    int height;
    int range;
    int stop_threshold;
    int same_threshold;

    int dst_width;
    int dst_height;

    bool has_mask; //no use
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
    // double pstd = 0.3;//4e-5 -Q: football many small person 4e-3 figure not severe jitter & object big move
    // double cstd = 0.15;//R:can be changed //tracker csrt scale 640 parameter with pre kalman
    // double pstd = 0.2;//4e-5 -Q: football many small person 4e-3 figure not severe jitter & object big move
    // double cstd = 0.05;//R:can be changed //tracker csrt scale 640 parameter with post kalman
    // double pstd = 0.01;//4e-5 -Q: football many small person 4e-3 figure not severe jitter & object big move
    // double cstd = 0.15;//R:can be changed //tracker csrt scale 640 parameter with post kalman
    double pstd = 0.005;//4e-5 -Q: football many small person 4e-3 figure not severe jitter & object big move
    double cstd = 0.2;//R:can be changed //tracker csrt scale 640 parameter with post kalman


	Trajectory X;//posteriori state estimate
	Trajectory X_;//priori estimate
	Trajectory P;// posteriori estimate error covariance
	Trajectory P_;// priori estimate error covariance
	Trajectory K;//gain
	Trajectory	z;//actual measurement
	Trajectory Q;// process noise covariance
	Trajectory R;// measurement noise covariance 

    ofstream out_transform;
    ofstream out_trajectory;
    ofstream out_smoothed;
    ofstream out_new;
    ofstream out_transform2;    
    ofstream out_trajectory2;
    ofstream out_smoothed2;
    ofstream out_new2;

} KALMAN;

int stab_2dof(char* in, char* out, int coord[4]);
int stab_live(char* infile);
int stab_pathsmoothe(char* infile);
//int test_search();
}