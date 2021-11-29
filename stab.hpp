
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

#include "common/TimeUtil.hpp"

using namespace std;
using namespace cv;

#define Q1 0.004
#define R1 0.2
static double sum_scaleX = 0;
static double sum_scaleY = 0;
static double sum_thetha = 0;
static double sum_transX = 0;
static double sum_transY = 0;
static double scaleX = 0;
static double scaleY = 0;
static double thetha = 0;
static double transX = 0;
static double transY = 0;
static double diff_scaleX = 0;
static double diff_scaleY = 0;
static double diff_transX = 0;
static double diff_transY = 0;
static double diff_thetha = 0;
static double errscaleX = 1;
static double errscaleY = 1;
static double errthetha = 1;
static double errtransX = 1;
static double errtransY = 1;

static double Q_scaleX = Q1;
static double Q_scaleY = Q1;
static double Q_thetha = Q1;
static double Q_transX = Q1;
static double Q_transY = Q1;

static double R_scaleX = R1;
static double R_scaleY = R1;
static double R_thetha = R1;
static double R_transX = R1;
static double R_transY = R1;


typedef struct _win_result {
    int glb_x;
    int glb_y;
    int loc_x;
    int loc_y;
    int min_sum_diff;

    void copy(_win_result* a, const _win_result* b) {
        a->glb_x = b->glb_x;
        a->glb_y = b->glb_y;
        a->loc_x = b->loc_x;
        a->loc_y = b->loc_y;
        a->min_sum_diff = b->min_sum_diff;
    }

} WIN_RESULT;

int stab_6dof(char* in, char* out);
int stab_2dof(char* in, char* out);
int stab_fastwin(char* in, char* out);

int MakeMask6(Mat& mask, int width, int height);
int MakeMask2(Mat& mask, int width, int height);
void Kalman_Filter(double *scaleX , double *scaleY , double *thetha , double *transX , double *transY);

Mat PickArea(Mat& src, int x ,int y, int winsize, int range);
int cvt_coord_to_vstmap(int sx, int sy, int range, int dx, int dy, int* tx, int ty);
int GetImageSum(Mat& itg, int xx, int yy, int x, int y);
int Search(Mat& src1, Mat& src2, int range, WIN_RESULT* win_result);
