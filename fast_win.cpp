
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

int stab_fastwin(char* in, char* out, int coord[4]) {

    VideoCapture stab(in);
    VideoWriter output;
    WIN_INFO* t_win = new WIN_INFO();    
    WIN_INFO* q_win = new WIN_INFO();
    t_win->init();
    q_win->init();

    char filename[30];
    int result = -1;
    int range = 31;

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
        Mat temp;
        resize(src1oc, src1o, Size(int((float)src1oc.cols/scale), int(float(src1oc.rows)/scale)), 0,0,1);
        cvtColor(src1o, temp, COLOR_BGR2GRAY);
        GaussianBlur(temp, src1o, {11, 11}, 0.9, 0.9);        

        if( i == 0) {
            PickArea(src1o, coord, range, t_win);
            // sprintf(filename, "%d_winmat0.png", i);
            // imwrite(filename, src2);
            i++;
            continue;
        }

        PickArea(src1o, coord, range, q_win);        
        Search(t_win, q_win, range);

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
        t_win = q_win;
       
        Logger("[%d] %f ", i, LapTimer(all));
    }

    delete t_win;
    delete q_win;

    return 1;
}

int PickArea(Mat& src, int coord[4], int range, WIN_INFO* _info) {
    _info->glb_x = coord[0];
    _info->glb_y = coord[1];
    _info->width = coord[2];    
    _info->height = coord[3];
    _info->loc_x = _info->glb_x + _info->width;
    _info->loc_y = _info->glb_y + _info->height;

    _info->srch_x = _info->width + range;
    _info->srch_y = _info->height + range;
    _info->tt_width =  _info->width + range * 2;
    _info->tt_height = _info->height + range *2;
    _info->min_dx = 0;
    _info->min_dy = 0;

    Rect rec = Rect(_info->glb_x - range, _info->glb_y - range, _info->width + (range *2 ), _info->height +(range *2));
    Mat pick = src(rec);
    Mat pickitg;
    integral(pick, pickitg);
    _info->itg = pickitg;
    return 1;
}

int cvt_win_to_vstmap(int sx, int sy, int range, int dx, int dy, int* tx, int ty) {

    return 1;
}

int GetImageSum(Mat& itg, int xx, int yy, int x, int y) {
    int sum = itg.at<int>(yy, xx) + itg.at<int>(y,x) - itg.at<int>(yy, xx) - itg.at<int>(yy, x);
    return sum;
}

int Search(WIN_INFO* t_win, WIN_INFO* q_win, int range) {

    int* vst_map = (int *)malloc(sizeof(unsigned char) *  t_win->tt_width * t_win->tt_height);
    memset(vst_map, 0, sizeof(unsigned char) *  t_win->tt_width * t_win->tt_height);
    int t_sum = GetImageSum(t_win->itg, t_win->srch_x, t_win->srch_y, t_win->srch_x - t_win->width, t_win->srch_y - t_win->height);
    Recursive(t_sum, q_win->srch_x, q_win->srch_y, vst_map, q_win);

    free(vst_map);
    return 0;
}

int Recursive(int t_sum, int _x, int _y, int* vst_map, WIN_INFO* _win) {

    int stepx[9] = {0, -1, -1,  0,  1, 1, 1, 0, -1};
    int stepy[9] = {0,  0, -1, -1, -1, 0, 1, 1,  1};
    int width = _win->width;

    for(int i = 0; i < sizeof(stepx)/sizeof(stepx[0]) ; i ++) {
        int newx = _x + stepx[i];
        int newy = _y + stepy[i];
        if(vst_map[ newy * width + newx] == 0 ) {
            vst_map[ newy * width + newx ] = 1;            
            int q_sum = GetImageSum(_win->itg, newx, newy, newx - _win->width, newy - _win->height);

            if(t_sum == q_sum) {
                Logger("Same point is found! %d %d ", newx, newy);
                _win->min_dx = newx;
                _win->min_dy = newy;
                _win->min_sum_diff = 0;
                return 1;
            } else if (abs(t_sum - q_sum) < _win->min_sum_diff) {
                _win->min_dx = newx - _win->srch_x;
                _win->min_dy = newy - _win->srch_y;
                _win->min_sum_diff = abs(t_sum - q_sum);
            }
        }
    }
           
    Recursive(t_sum, _x + stepx[1], _y + stepy[1], vst_map, _win );
} 