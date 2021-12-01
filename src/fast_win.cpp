
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
    PARAM* p = new PARAM();
    WIN_INFO* t_win = new WIN_INFO();    
    WIN_INFO* q_win = new WIN_INFO();
    t_win->init();
    q_win->init();
    p->scale = 2;
    p->sx = coord[0];
    p->sy = coord[1];
    p->width = coord[2];
    p->height = coord[3];    
    p->range = 31;
    p->blur_size = 11;
    p->blur_sigma = 0.9;

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
        resize(src1oc, src1o, Size(int((float)src1oc.cols/p->scale), int(float(src1oc.rows)/p->scale)), 0,0,1);
        cvtColor(src1o, temp, COLOR_BGR2GRAY);
        GaussianBlur(temp, src1o, {p->blur_size, p->blur_size}, p->blur_sigma, p->blur_sigma);

        sprintf(filename, "%d_src.png", i);
        imwrite(filename, src1o);

        if( i == 0) {
            PickArea(src1o, t_win, p);
            i++;
            continue;
        }

        PickArea(src1o, q_win, p);        
        ShowData(q_win, p);

        Search(t_win, q_win, p);
        if(i == 2)
            break;
        
        smth.at<double>(0,0) = 1; //ds_x * cos(da);
        smth.at<double>(0,1) = 0; //ds_x * -sin(da);
        smth.at<double>(1,0) = 0; //ds_y * sin(da);
        smth.at<double>(1,1) = 1; //ds_y * cos(da);
        smth.at<double>(0,2) = dx;
        smth.at<double>(1,2) = dy;

        warpAffine(src1, src1, smth, src1.size());        
        smth.at<double>(0,2) = dx * p->scale;
        smth.at<double>(1,2) = dy * p->scale;      

        warpAffine(src1oc, src1oc, smth, src1oc.size());
        output << src1oc;
        t_win = q_win;
        i++;
        
        Logger("[%d] %f ", i, LapTimer(all));
    }

    delete p;
    delete t_win;
    delete q_win;

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
    _info->tt_height = _info->height + (p->range) *2;
    _info->min_dx = 0;
    _info->min_dy = 0;

    printf("Pick -- %d %d %d %d \n", _info->loc_x, _info->loc_y, _info->tt_width, _info->tt_height);
    Rect rec = Rect(_info->loc_x, _info->loc_y, _info->tt_width, _info->tt_height);
    Mat pick = src(rec);
    Mat pickitg;
    integral(pick, pickitg, CV_32S);
    _info->itg = pickitg;

    //ShowPickArea(_info);
    return 1;
}

int cvt_win_to_vstmap(int sx, int sy, int range, int dx, int dy, int* tx, int ty) {

    return 1;
}

int GetImageSum(Mat& itg, int xx, int yy, int x, int y) {
    printf("GetImageSum %d %d %d %d \n", xx, yy ,x , y);
    int sum = itg.at<int>(yy, xx) + itg.at<int>(y,x) - itg.at<int>(yy, xx) - itg.at<int>(yy, x);
    return sum;
}

int Search(WIN_INFO* t_win, WIN_INFO* q_win, PARAM* p) {

    int* vst_map = (int *)malloc(sizeof(int) *  t_win->tt_width * t_win->tt_height);
    memset(vst_map, 0, sizeof(int) *  t_win->tt_width * t_win->tt_height);
    int t_sum = GetImageSum(t_win->itg, t_win->srch_x, t_win->srch_y, t_win->srch_x - t_win->width, 
        t_win->srch_y - t_win->height);
    SpiralSearch(t_sum, q_win->srch_x, q_win->srch_y, vst_map, q_win, p);
    Logger("Search Result min_diff %d dx %d dy %d" , q_win->min_sum_diff, q_win->min_dx, q_win->min_dy);

    free(vst_map);
    return 0;
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

    for (int k = 0; k < np; ++k) {
        // make a step, add 'direction' vector (di, dj) to current position (i, j)
        i += di;
        j += dj;
        ++segment_passed;

        int newx = _x + i;
        int newy = _y + j;
        printf("[%d] -- %d  %d  --  %d %d \n", k, i , j , newx, newy);
        vst_map[ newy * _win->tt_width + newx] = 1;
        
        int q_sum = GetImageSum(_win->itg, newx, newy, newx - _win->width,  newy - _win->height);
        if(t_sum == q_sum) {
            Logger("Same point is found! %d %d ", newx, newy);
            _win->min_dx = newx;
            _win->min_dy = newy;
            _win->min_sum_diff = 0;
            break;

        } else if (abs(t_sum - q_sum) < _win->min_sum_diff) {
            Logger("minimum point update! %d %d ", newx, newy);
            _win->min_dx = newx;
            _win->min_dy = newy;
            _win->min_sum_diff = abs(t_sum - q_sum);
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

    ShowVisitMap(vst_map, _win->tt_width, _win->tt_height);
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
} 

void ShowData(WIN_INFO* _win, PARAM* _p) {
    printf(" ==== PARAM  === \n");    
    printf("scale %d  \n", _p->scale);
    printf("start x %d y %d  \n", _p->sx, _p->sy);
    printf("area width %d  height %d \n", _p->width, _p->height);

    printf(" ==== WIN INFO === \n");
    printf(" glb _x %d _y %d \n", _win->glb_x, _win->glb_y);
    printf(" loc _x %d _y %d \n", _win->loc_x, _win->loc_y);    
    printf(" width %d height %d  \n", _win->width, _win->height);
    printf(" srch _x %d _y %d \n", _win->srch_x, _win->srch_y);    
    printf(" total width %d height %d  \n", _win->tt_width, _win->tt_height);    

/* -- integral imag e
    for(int i = 0; i < _win->tt_height ; i++) {
        for(int j = 0 ; j < _win->tt_width ; j ++) {
            printf(" %8d ", _win->itg.at<int>(i, j));
        }
        printf("\n [%d]", i);
    }
*/
}

void ShowVisitMap(int* vst, int width, int height) {

    for(int i = 51 ; i < width; i ++) {
        for(int j = 51 ; j < height ; j ++){
            printf("%d ", vst[j * width + i]);
        }
        printf("\n");
    }
}