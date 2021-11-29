
/*****************************************************************************
*                                                                            *
*                           fast_win          								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : fast_win.cpp
    Author(S)       : Me Eunkyung
    Created         : 28 nov 2021

    Description     : fast_win.cpp
    Notes           : 2dof video stabilization with integral image search
*/

#include "stab.hpp"

int stab_fastwin(char* in, char* out) {

    VideoCapture stab(in);
    VideoWriter output;
    WIN_RESULT* cal_result = new WIN_RESULT();    
    WIN_RESULT* cal_result_pre = new WIN_RESULT();


    char filename[30];
    int result = -1;
    int range = 21;
    int win = 20;
    int initial_x = 600; //after sacle
    int initial_y = 105;
    int x = initial_x;
    int y = initial_y;

    Mat src1; Mat src1oc; Mat src1o;
    Mat winmat;
    Mat winitg;    
    Mat src2;

    Mat smth;
    Mat pre_affine;
    Mat affine;
    int scale = 2;
    int i = 0;
    int dx = 0;
    int dy = 0;

    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    
    smth.create(2 , 3 , CV_64F);    

    while(true) {

        stab >> src1oc;
        if(src1oc.data == NULL)
            break;

        resize(src1oc, src1o, Size(int((float)src1oc.cols/scale), int(float(src1oc.rows)/scale)), 0,0,1);
        cvtColor(src1o, src1o, COLOR_BGR2GRAY);

        if( i == 0) {
            winitg = PickArea(src1o, initial_x , initial_y , win, range);
            src2 = winitg;
            // sprintf(filename, "%d_winmat0.png", i);
            // imwrite(filename, src2);
            i++;
            continue;
        }

        winitg = PickArea(src1o, x , y, win, range);            
        winitg.copyTo(src1);
        
        Search(src1, src2, range, cal_result);

        smth.at<double>(0,0) = 1; //ds_x * cos(da);
        smth.at<double>(0,1) = 0; //ds_x * -sin(da);
        smth.at<double>(1,0) = 0; //ds_y * sin(da);
        smth.at<double>(1,1) = 1; //ds_y * cos(da);
        smth.at<double>(0,2) = dx;
        smth.at<double>(1,2) = dy;

        warpAffine(src1, src1, smth, src1.size());        
        smth.at<double>(0,2) = dx * scale;
        smth.at<double>(1,2) = dy * scale;      

        warpAffine(src1oc, src1oc, smth, src1oc.size());
        output << src1oc;
        src1.copyTo(src2);     
        cal_result_pre = cal_result;        
       
        Logger("[%d] %f ", i, LapTimer(all));
    }

    delete cal_result;
    delete cal_result_pre;

    return 1;
}

Mat PickArea(Mat& src, int x ,int y, int winsize, int range) {
    Rect rec = Rect(x - range, y - range, winsize + range*2, winsize+ range*2);
    Mat pick = src(rec);
    Mat pickitg;
    integral(pick, pickitg);
    return pickitg;
}

int cvt_win_to_vstmap(int sx, int sy, int range, int dx, int dy, int* tx, int ty) {

    return 1;
}

int GetImageSum(Mat& itg, int xx, int yy, int x, int y) {
    int sum = itg.at<CV_32SC1>(yy, xx) + itg.at<CV_32SC1>(y,x) - itg.at<CV_32SC1>(yy, xx) - itg.at<CV_32SC1>(yy, x);
    return sum;
}

int Search(Mat& src1, Mat& src2, int range, WIN_RESULT* win_result) {

    int sx = 0; int sy = 0;
    int cx = sx + range/2; int cy = sy + range/2;
    int kernel = 0;
    int* vst_map = (int *)malloc(sizeof(int) * range * range );
    memset(vst_map, 0, sizeof(int) * range * range);
    int step_cnt = 8;
    int stepx[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
    int stepy[8] = { 0, -1, -1, -1, 0, 1, 1,  1};
    sx = cx;
    sy = cy;
    for(int i = 0; i < range/2; i++) {
        kernel = i * 2 + 1;
        if (kernel == 1) {
            
        } else {

        }

    }

    free(vst_map);
    return 0;
}