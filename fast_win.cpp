
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
    WIN_INFO* cal_info = new WIN_INFO();    
    WIN_INFO* cal_info_pre = new WIN_INFO();


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
        
        Search(src1, src2, range, cal_info);

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
        cal_info_pre = cal_info;
       
        Logger("[%d] %f ", i, LapTimer(all));
    }

    delete cal_info;
    delete cal_info_pre;

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
    int sum = itg.at<int>(yy, xx) + itg.at<int>(y,x) - itg.at<int>(yy, xx) - itg.at<int>(yy, x);
    return sum;
}

int Search(Mat& src1, Mat& src2, int range, WIN_INFO* win_info) {

    int sx = 0; int sy = 0;
    int cx = sx + range/2; int cy = sy + range/2;
    int kernel = 0;
    int* vst_map = (int *)malloc(sizeof(int) * range * range );
    memset(vst_map, 0, sizeof(int) * range * range);
    int step_cnt = 8;
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

int Recursive(int t_sum, int anc_x, int anc_y, int* vst_map, Mat& itg, WIN_INFO* win_info) {

    int stepx[9] = {0, -1, -1,  0,  1, 1, 1, 0, -1};
    int stepy[9] = {0,  0, -1, -1, -1, 0, 1, 1,  1};
    int width = win_info->width;

    for(int i = 0; i < sizeof(stepx)/sizeof(stepx[0]) ; i ++) {
        int newx = anc_x + stepx[i];
        int newy = anc_y + stepy[i];
        if(vst_map[ newy * width + newx] == 0 ) {
            vst_map[ newy * width + newx ] = 1;            
            int q_sum = GetImageSum(itg, newx, newy, newx - win_info->width, newy - win_info->height);

            if(t_sum == q_sum) {
                //recored result
                return 1;
            } else if (abs(t_sum - q_sum) < win_info->min_sum_diff) {
                win_info->loc_x = anc_x + stepx[i];
                win_info->loc_y = anc_y + stepy[i];
            }
        }
    }
           
    Recursive( t_sum, anc_x + stepx[1], anc_y + stepy[1], vst_map, itg, win_info );
}