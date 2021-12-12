
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

#pragma once
#include "DefData.hpp"
#include "Detection.hpp"

using namespace std;
using namespace cv;

struct TransformParam
{
    TransformParam() {}
    TransformParam(double _dx, double _dy, double _da) {
        dx = _dx;
        dy = _dy;
        da = _da;
    }

    double dx;
    double dy;
    double da; // angle
};

struct Trajectory
{
    Trajectory() {}
    Trajectory(double _x, double _y, double _a) {
        x = _x;
        y = _y;
        a = _a;
    }
	// "+"
	friend Trajectory operator+(const Trajectory &c1,const Trajectory  &c2){
		return Trajectory(c1.x+c2.x,c1.y+c2.y,c1.a+c2.a);
	}
	//"-"
	friend Trajectory operator-(const Trajectory &c1,const Trajectory  &c2){
		return Trajectory(c1.x-c2.x,c1.y-c2.y,c1.a-c2.a);
	}
	//"*"
	friend Trajectory operator*(const Trajectory &c1,const Trajectory  &c2){
		return Trajectory(c1.x*c2.x,c1.y*c2.y,c1.a*c2.a);
	}
	//"/"
	friend Trajectory operator/(const Trajectory &c1,const Trajectory  &c2){
		return Trajectory(c1.x/c2.x,c1.y/c2.y,c1.a/c2.a);
	}
	//"="
	Trajectory operator =(const Trajectory &rx){
		x = rx.x;
		y = rx.y;
		a = rx.a;
		return Trajectory(x,y,a);
	}

    double x;
    double y;
    double a; // angle
};

class Dove {

public: 
    PARAM* p;
    TIMER* t;
    Dlog dl;
    Detection dt;

    string _in;
    string _out;

    Mat mask;
    Mat src1;
    Mat src2;
    Mat smth;
    Mat last_smth;
    Mat ref;

    Mat pre_affine;
    Mat affine;
    int cp_width = 0;
    int cp_height = 0;
    char filename[30];
    int i = 0;
    int threshold = 6;

    //KALMAN* k;
    double x = 0;
    double y = 0;
    double a = 0; // angle    
    vector <TransformParam> prev_to_cur_transform; // previous to current 
    vector <Trajectory> trajectory; // trajectory at all frames           
	vector <Trajectory> smoothed_trajectory; // trajectory at all frames
	Trajectory X;//posteriori state estimate
	Trajectory	X_;//priori estimate
	Trajectory P;// posteriori estimate error covariance
	Trajectory P_;// priori estimate error covariance
	Trajectory K;//gain
	Trajectory	z;//actual measurement
	Trajectory Q;
	Trajectory R;
	vector <TransformParam> new_prev_to_cur_transform;    

    Dove(string infile, string outfile);
    Dove(int mode, bool has_mask, int* coord, string infile, string outfile, string id = "TEST");
    ~Dove();
    int Process();

    int ImageProcess(Mat& src, Mat& dst);
    void SetRef(Mat& _src) {_src.copyTo(ref); };
    void Initialize(bool has_mask, int* coord);
    int CalculateMove(Mat& cur);

    int Detect(Mat& cur);

    int CalculateMove_LK(Mat& cur);
    int CalculateMove_Integral(Mat& cur);
    int CalculateMove_Tracker(Mat& cur);

    int MakeMask();
    int ApplyImage(Mat& src, bool scaled = false);

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