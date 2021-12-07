
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

#include "DefData.hpp"

int stab_fastwin(char* in, char* out, int coord[4]) {

    VideoCapture stab(in);
    VideoWriter output;
    output.open(out, VideoWriter::fourcc('A', 'V', 'C', '1'), 30, Size(1920, 1080));

    WIN_INFO t; WIN_INFO q;
    PARAM pr;
    WIN_INFO* t_win = &t;
    WIN_INFO* q_win = &q;
    PARAM* p =&pr;

    t_win->init();
    q_win->init();
    p->scale = 1;
    p->sx = coord[0];
    p->sy = coord[1];
    p->width = coord[2];
    p->height = coord[3];    
    p->range = 10;
    p->blur_size = 11;
    p->blur_sigma = 1.3;
    p->stop_threshold = 50;
    p->same_threshold = 100;

    char filename[30];
    int result = -1;

    Mat src1; Mat src1oc; Mat src1o;
    Mat winmat;
    Mat winitg;    
    Mat src2;

    Mat smth;
    Mat pre_affine;
    Mat affine;
    int i = 0;
    double dx  = 0;
    double ddx = 0;
    double dy = 0;
    double ddy = 0;

    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    
    smth.create(2 , 3 , CV_64F);    

    while(true) {

        stab >> src1oc;
        if(src1oc.data == NULL)
            break;
        Mat temp;
        if(p->scale != 1)
            resize(src1oc, src1o, Size(int((float)src1oc.cols/p->scale), int(float(src1oc.rows)/p->scale)), 0,0,1);
        else
            src1oc.copyTo(src1o);
        cvtColor(src1o, temp, COLOR_BGR2GRAY);
        GaussianBlur(temp, src1o, {p->blur_size, p->blur_size}, p->blur_sigma, p->blur_sigma);


        if( i == 0) {
            sprintf(filename, "%d_pro_o.png", i);
            imwrite(filename, src1o);
            
            PickArea(src1o, t_win, p);
            // sprintf(filename, "%d_out.png", i);
            // imwrite(filename, src1oc);

            i++;
            continue;
        }

        PickArea(src1o, q_win, p);        
        result = Search(t_win, q_win, p);        
        if( result > p->same_threshold ) {
            Logger("can't find base point for stabilization.. action TBD ");
            t_win = q_win;            
            i++;
            continue;
        }
        ddx = (q_win->srch_x - q_win->min_dx);
        ddy = (q_win->srch_y - q_win->min_dy);
        printf("ddx %f ddy %f \n ", ddx, ddy);
        if( ddx < 1 && ddy < 1) {
            InfoMove(t_win, q_win);
            output << src1oc;                        
            i++;
            continue;
        }

        dx = (q_win->srch_x - q_win->min_dx) * p->scale * 0.6;
        dy = (q_win->srch_y - q_win->min_dy) * p->scale * 0.6;


        smth.at<double>(0,0) = 1; //ds_x * cos(da);
        smth.at<double>(0,1) = 0; //ds_x * -sin(da);
        smth.at<double>(1,0) = 0; //ds_y * sin(da);
        smth.at<double>(1,1) = 1; //ds_y * cos(da);
        smth.at<double>(0,2) = dx;
        smth.at<double>(1,2) = dy;

        //warpAffine(src1, src1, smth, src1.size());        
        //smth.at<double>(0,2) = dx * p->scale;
        //smth.at<double>(1,2) = dy * p->scale;      
        // sprintf(filename, "%d_ori.png", i);
        // imwrite(filename, src1oc);
        // sprintf(filename, "%d_pro_o.png", i);
        // imwrite(filename, src1o);

        warpAffine(src1oc, src1oc, smth, src1oc.size());
        output << src1oc;
        // sprintf(filename, "%d_out.png", i);
        // imwrite(filename, src1oc);

        InfoMove(t_win, q_win);
        dx = 0; dy = 0;
        i++;
        if(i == 30)
            break;
        
        Logger("[%d] %f ", i, LapTimer(all));
    }

    return 1;
}

int PickArea(Mat& src, WIN_INFO* _info, PARAM* p) {    
    _info->glb_x = p->sx / p->scale;
    _info->glb_y = p->sy / p->scale;
    _info->width = p->width / p->scale;    
    _info->height = p->height / p->scale;
    _info->loc_x = _info->glb_x - p->range;
    _info->loc_y = _info->glb_y - p->range;

    _info->srch_x = _info->width + p->range;
    _info->srch_y = _info->height + p->range;
    _info->tt_width =  _info->width + (p->range) * 2;
    _info->tt_height = _info->height + (p->range) * 2;
    _info->min_dx = 0;
    _info->min_dy = 0;

    _info->min_sum_diff = 255 * _info->width * _info->height;

    _info->itg.release();
    Rect rec = Rect(_info->loc_x, _info->loc_y, _info->tt_width, _info->tt_height);
    Mat pick = src(rec);
    Mat pickitg;
    integral(pick, pickitg, CV_32S);
    _info->itg = pickitg;

    char filename[30];
    static int i = 0;
    sprintf(filename, "%d_pick.png", i);
    imwrite(filename, pick);
    i++;

    //ShowData(_info, p, 1);
    //ShowData(_info, p, 2);    

    return 1;
}

int GetImageSum(Mat& itg, int xx, int yy, int x, int y) {
    int sum = itg.at<int>(yy, xx) + itg.at<int>(y,x) - itg.at<int>(y, xx) - itg.at<int>(yy, x);
//    printf("itg address %p \n", &itg);
//    printf("Get Image Sum %d %d %d %d \n", itg.at<int>(yy, xx), itg.at<int>(y,x),itg.at<int>(y, xx), itg.at<int>(yy, x));
    return sum;
}

int Search(WIN_INFO* t_win, WIN_INFO* q_win, PARAM* p) {

    int* vst_map = (int *)malloc(sizeof(int) *  t_win->tt_width * t_win->tt_height);
    memset(vst_map, 0, sizeof(int) *  t_win->tt_width * t_win->tt_height);

    int t_sum = GetImageSum(t_win->itg, t_win->srch_x, t_win->srch_y, t_win->srch_x - t_win->width, 
        t_win->srch_y - t_win->height);
    int result = SpiralSearch(t_sum, q_win->srch_x, q_win->srch_y, vst_map, q_win, p);
    Logger("Search Result min_diff %d dx %d dy %d" , q_win->min_sum_diff, q_win->min_dx, q_win->min_dy);
    //ShowVisitMap(vst_map, q_win->tt_width, q_win->tt_height);

    free(vst_map);
    return result;
}

int SpiralSearch(int t_sum, int _x, int _y, int* vst_map, WIN_INFO* _win, PARAM* p)
{
    int np = (p->range * 2) * (p->range * 2);
    int di = 1;
    int dj = 0;
    int segment_length = 1;
    // current position (i, j) and how much of current segment we passed
    int i = 0;
    int j = 0;
    int segment_passed = 0;

    int q_sum = GetImageSum(_win->itg, _x, _y, _x - _win->width,  _y - _win->height);
    //Logger("t_sum %d q_sum %d ", t_sum, q_sum);
    vst_map[ _y * _win->tt_width + _x] = 1;    
    if(t_sum == q_sum) {
        Logger("Same point is found! -- 1  %d %d ", _x, _y);
        _win->min_dx = _x;
        _win->min_dy = _y;
        _win->min_sum_diff = 0;
        return 100;
    }

    for (int k = 0; k < np; ++k) {
        // make a step, add 'direction' vector (di, dj) to current position (i, j)
        i += di;
        j += dj;
        ++segment_passed;

        int newx = _x + i;
        int newy = _y + j;
        vst_map[ newy * _win->tt_width + newx] = 1;        
        int q_sum = GetImageSum(_win->itg, newx, newy, newx - _win->width,  newy - _win->height);
        //printf("[%d] -- %d  %d  --  %d %d --- tsum %d : qsum %d \n", k, i , j , newx, newy, t_sum, q_sum);

        if(t_sum == q_sum) {
            Logger("Same point is found! -- 2 [%d, %d] t_sum %d q_sum %d iter[%d]", newx, newy, t_sum, q_sum, k);
            _win->min_dx = newx;
            _win->min_dy = newy;
            _win->min_sum_diff = 0;
            return 100;

        } else if (abs(t_sum - q_sum) < _win->min_sum_diff) {
            //Logger("minimum point update! sum %d [%d, %d] t_sum %d q_sum %d iter[%d]", 
            //    _win->min_sum_diff, newx, newy, t_sum, q_sum, k);
            _win->min_dx = newx;
            _win->min_dy = newy;
            _win->min_sum_diff = abs(t_sum - q_sum);

            if (_win->min_sum_diff < p->stop_threshold) {
                Logger("minimum point is lower than threshold ..!");
                return 10;
            }
        }

        if (segment_passed == segment_length) {
            // done with current segment
            segment_passed = 0;

            // 'rotate' directions
            int buffer = di;
            di = -dj;
            dj = buffer;

            // increase segment length if necessary
            if (dj == 0) {
                ++segment_length;
            }
        }
    }

    return -1;
}

void InfoMove(WIN_INFO* t, WIN_INFO* q) {
    t->glb_x = q->glb_x;
    t->glb_y = q->glb_y;    
    t->loc_x = q->loc_x;
    t->loc_y = q->loc_y;    
    t->range = q->range;
    t->width = q->width;
    t->height = q->height;
    t->srch_x = q->srch_x;
    t->srch_y = q->srch_y;
    t->tt_width = q->tt_width;
    t->tt_height = q->tt_height;
    t->itg.release();
    t->itg = q->itg.clone();
}

void ShowData(WIN_INFO* _win, PARAM* _p, int mode) {
    if(mode == 1) {
        printf(" ==== PARAM  === \n");    
        printf("scale %f  \n", _p->scale);
        printf("start x %d y %d  \n", _p->sx, _p->sy);
        printf("area width %d  height %d \n", _p->width, _p->height);

        printf(" ==== WIN INFO === \n");
        printf(" glb _x %d _y %d \n", _win->glb_x, _win->glb_y);
        printf(" loc _x %d _y %d \n", _win->loc_x, _win->loc_y);    
        printf(" width %d height %d  \n", _win->width, _win->height);
        printf(" srch _x %d _y %d \n", _win->srch_x, _win->srch_y);    
        printf(" total width %d height %d  \n", _win->tt_width, _win->tt_height);    
    } 
    else if( mode == 2) {

        for(int i = 81 - 5; i < 81 + 5 ; i++) {
            for(int j = 81 -5 ; j < 81 +5 ; j ++) {
                printf("[%d] %7d ", i, _win->itg.at<int>(i, j));
            }
            printf("\n", i);
        }
    }
}

void ShowVisitMap(int* vst, int width, int height) {

    for(int i = 51 ; i < width; i ++) {
        for(int j = 51 ; j < height ; j ++){
            printf("%d ", vst[j * width + i]);
        }
        printf("\n");
    }
}


int Recursive(int t_sum, int _x, int _y, int* vst_map, WIN_INFO* _win, PARAM* p, int a) {

    int stepx[5] = {0, -1, -1, 1, 0};
    int stepy[5] = {0,  0, -1, 1, 1};
    int width = _win->tt_width;

    if(_x < _win->srch_x - p->range || _x > _win->srch_x + p->range ||
       _y < _win->srch_y - p->range || _y > _win->srch_y + p->range) {
        printf("ragne over. finish. \n");
        return 1;
    }

    for(int i = 0; i < sizeof(stepx)/sizeof(stepx[0]) ; i ++) {
        int newx = _x + stepx[i];
        int newy = _y + stepy[i];
        printf("newx %d newy %d vst %d  \n ", newx, newy, vst_map[ newy * width + newx]);

        if(vst_map[ newy * width + newx] == 0 ) {
            vst_map[ newy * width + newx ] = 1;            
            int q_sum = GetImageSum(_win->itg, newx, newy, newx - _win->width,  newy - _win->height);

            if(t_sum == q_sum) {
                //Logger("Same point is found! %d %d ", newx, newy);
                _win->min_dx = newx;
                _win->min_dy = newy;
                _win->min_sum_diff = 0;
                //return 1;

            } else if (abs(t_sum - q_sum) < _win->min_sum_diff) {
                _win->min_dx = newx - _win->srch_x;
                _win->min_dy = newy - _win->srch_y;
                _win->min_sum_diff = abs(t_sum - q_sum);
            }
        }
    }
    if (a < sizeof(stepx)/sizeof(stepx[0])) {
        a++;
        Recursive(t_sum, _x + stepx[a], _y + stepy[a], vst_map, _win, p, a);
    }

    printf("---- for loop close ---- \n");           
    return 1;
} 
