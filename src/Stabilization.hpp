
/*****************************************************************************
*                                                                            *
*                           Stabilization.hpp  								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Stabilization.cpp
    Author(S)       : Me Eunkyung
    Created         : 07 dec 2021

    Description     : Stabilization.cpp
    Notes           : Stabilization main class
*/


#include "DefData.hpp"

using namespace std;
using namespace cv;

class Dove {

public: 
    PARAM* p;
    TIMER* t;
    Dlog dl;

    Mat src1; Mat src1oc; Mat src1o;
    Mat mask;
    Mat src2;
    Mat smth;

    Mat pre_affine;
    Mat affine;
    int cp_width = 0;
    int cp_height = 0;
    char filename[30];
    int i = 0;
    int threshold = 6;


    Dove();
    Dove(bool has_mask, int* coord, string id = "TEST");
    ~Dove();

    int Process();
    void Initialize(int* coord, bool has_mask);
    int CalculateMove(Mat& src1, Mat& src2, Mat& smth);
    int MakeMask(Mat& mask, PARAM* p);
    int ApplyImage(Mat& src, Mat& dst, Mat& smth);

    int stab_2dof(char* in, char* out, int coord[4]);
    int stab_fastwin(char* in, char* out, int coord[4]);
    int stab_6dof(char* in, char* out);

    // void Kalman_Filter(double *scaleX , double *scaleY , double *thetha , double *transX , double *transY);

    int PickArea(Mat& src, WIN_INFO* _info, PARAM* p);
    int cvt_coord_to_vstmap(int sx, int sy, int range, int dx, int dy, int* tx, int ty);
    int GetImageSum(Mat& itg, int xx, int yy, int x, int y);
    int Search(WIN_INFO* t_win, WIN_INFO* q_win, PARAM* p);
    int RecursiveParent(int t_sum, int _x, int _y, int* vst_map, WIN_INFO* _win, PARAM* p);
    int Recursive(int t_sum, int anc_x, int anc_y, int* vst_map, WIN_INFO* win_info, PARAM* p, int a);
    int SpiralSearch(int t_sum, int _x, int _y, int* vst_map, WIN_INFO* _win, PARAM* p);
    void InfoMove(WIN_INFO* t, WIN_INFO* q);

    // void ShowData(WIN_INFO* _win, PARAM* _p, int mode );
    // void ShowVisitMap(int* vst, int width, int height);

};