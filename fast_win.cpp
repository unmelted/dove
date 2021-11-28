
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

    char filename[30];
    int result = -1;
    int range = 20;
    int win = 100;
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
        
        result = Search(src1, src2, range, &dx, &dy);

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

       
        Logger("[%d] %f ", i, LapTimer(all));
    }

    return 1;
}

Mat PickArea(Mat& src, int x ,int y, int winsize, int range) {
    Rect rec = Rect(x - range, y - range, winsize + range*2, winsize+ range*2);
    Mat pick = src(rec);
    Mat pickitg;
    integral(pick, pickitg);
    return pickitg;
}

int Search(Mat& src1, Mat& src2, int range, int* dx, int* dy) {
    int sx = 20; int sy = 20;
    
    return 1;
}