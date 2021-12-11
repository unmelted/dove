
/*****************************************************************************
*                                                                            *
*                           Stabilization.cpp  								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Stabilization.cpp
    Author(S)       : Me Eunkyung
    Created         : 07 dec 2021

    Description     : main procee for Stabilization
    Notes           : Stabilization main class
*/


#include "Stabilization.hpp"

using namespace std;
using namespace cv;

Dove::Dove(string infile, string outfile) {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
    dt = Detection();

    dl.SetLogFilename("TEST");
    _in = infile;
    _out = outfile;
    Initialize(false, 0);    

}

Dove::Dove(int mode, bool has_mask, int* coord, string infile, string outfile, string id) {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
    dt = Detection();    

    dl.Logger("instance created.. ");
    dl.SetLogFilename("TEST");
    _in = infile;
    _out = outfile;

    dl.Logger("Start construct. %s %s  ", _in.c_str(), _out.c_str());
    p->mode = mode;
    if(has_mask == true) 
        Initialize(true, coord);
    else 
        Initialize(false, 0);    
}

Dove::~Dove() {
    delete p;
    delete t;

    if(k != NULL)
        delete k;
}

int Dove::Process() {
    dl.Logger("process start..");

    VideoCapture in(_in);
    Mat src1oc; Mat src1o;
    int i = 0;
    int result = 0;

    while(true) {
        in >> src1oc;
        if(src1oc.data == NULL)
            break;
        
        ImageProcess(src1oc, src1o);

        if ( i == 0)
        {
            SetRef(src1o);
            i++;
            continue;
        }
    
        result = CalculateMove(src1o);
        ApplyImage(src1o);
        ApplyImage(src1oc, true);

        out << src1oc;        
        // char filename[40];
        // sprintf(filename, "saved/%d_warp.png", i);
        // imwrite(filename, src1oc);

        SetRef(src1o);        
        i++;
        // if(i == 20)
        //     break;
    }

    return ERR_NONE;
};

void Dove::Initialize(bool has_mask, int* coord) {
    p->scale = 2;
    if(has_mask == true) {
        p->has_mask = true;
        p->sx = coord[0] / p->scale;
        p->sy = coord[1] / p->scale;
        p->width = coord[2] / p->scale;
        p->height = coord[3] / p->scale;    
    }

    p->blur_size = 5;
    p->blur_sigma = 0.7;
    p->dst_width = 1920;
    p->dst_height = 1080;

    if(p->run_detection == true) {
        p->detector_type = DARKNET_YOLOV4;
        p->names_file = "darknet/data/coco.names";
        p->cfg_file = "darknet/cfg/yolov3.cfg";
        p->weights_file = "darknet/yolov3.weights";
        dt.LoadModel(p);
    }
    if(p->run_kalman == true) {
        k = new KALMAN();
        k->Q.set(k->pstd, k->pstd, k->pstd);
        k->R.set(k->cstd, k->cstd, k->cstd);        
        k->out_transform.open("prev_to_cur_transformation.txt");
        k->out_trajectory.open("trajectory.txt");
        k->out_smoothed.open("smoothed_trajectory.txt");
        k->out_new.open("new_prev_to_cur_transformation.txt");
    }

    smth.create(2 , 3 , CV_64F);        
    out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(p->dst_width, p->dst_height));

    dl.Logger("Initialized compelete.");    
}

int Dove::ImageProcess(Mat& src, Mat& dst) {
    Mat temp;
    if(p->scale != 1)
        resize(src, temp, Size(int((float)src.cols/p->scale), int(float(src.rows)/p->scale)), 0,0,1);

    cvtColor(temp, temp, COLOR_BGR2GRAY);
    GaussianBlur(temp, dst, {p->blur_size, p->blur_size}, p->blur_sigma, p->blur_sigma);

    if(p->has_mask)
        MakeMask();

    return ERR_NONE;
}

int Dove::CalculateMove(Mat& cur) {
    int result = -1;
    if(p->mode == OPTICALFLOW_LK_2DOF || p->mode == OPTICALFLOW_LK_6DOF) {
        result = CalculateMove_LK(cur);
    } else if (p->mode == INTEGRAL_IMAGE) {
        result = CalculateMove_Integral(cur);
    } else {
        result = CalculateMove_Tracker(cur);
    }
    return result;
}

int Dove::CalculateMove_LK(Mat& cur) {
    //static int i = 1;
    // sprintf(filename, "saved/%d_cur.png", i);
    // imwrite(filename, cur);
    // sprintf(filename, "saved/%d_ref.png", i);
    // imwrite(filename, ref);
    //i++;

    vector <Point2f> features1, features2;
    vector <Point2f> goodFeatures1, goodFeatures2;
    vector <uchar> status;
    vector <float> err;

    if(p->has_mask == true)
        goodFeaturesToTrack(cur, features1, 30, 0.01  , 30, mask, 11, false, 0.04);
    else 
        goodFeaturesToTrack(cur, features1, 30, 0.01  , 30, noArray(), 11, false, 0.04);    
    calcOpticalFlowPyrLK(cur, ref, features1, features2, status, err );

    for(size_t i=0; i < status.size(); i++)
    {
        if(status[i])
        {
            goodFeatures1.push_back(features1[i]);
            goodFeatures2.push_back(features2[i]);
        }
    }

    if(goodFeatures1.size() < threshold || goodFeatures2.size() < threshold) {
            dl.Logger("[%d] no feature to track.. feature %d: ", i,  goodFeatures1.size());
            pre_affine.copyTo(affine);
    }
    else {
        affine = estimateAffine2D(goodFeatures1, goodFeatures2);
        //affine = estimateRigidTransform(goodFeatures1, goodFeatures2, false);                
    }

    if(affine.empty() == true) {
        dl.Logger("there is no solution ..");
        pre_affine.copyTo(affine);
    }

    affine.copyTo(pre_affine);

    double dx = affine.at<double>(0,2);
    double dy = affine.at<double>(1,2);
    double da = atan2(affine.at<double>(1,0), affine.at<double>(0,0));
    double ds_x = affine.at<double>(0,0)/cos(da);
    double ds_y = affine.at<double>(1,1)/cos(da);

    if(p->run_kalman) {
        k->out_transform << i << " " << dx << " " << dy << " " << da << endl;        
		k->x += dx;
		k->y += dy;
		k->a += da;
		//trajectory.push_back(Trajectory(x,y,a));
		//
		k->out_trajectory << i << " " << k->x << " " << k->y << " " << k->a << endl;
		//
		k->z = Trajectory(k->x,k->y,k->a);
		//
		if(i == 1){
			// intial guesses
			k->X = Trajectory(0,0,0); //Initial estimate,  set 0
			k->P = Trajectory(1,1,1); //set error variance,set 1
		}
		else
		{
			//time update（prediction）
			k->X_ = k->X; //X_(k) = X(k-1);
			k->P_ = k->P+k->Q; //P_(k) = P(k-1)+Q;
			// measurement update（correction）
			k->K = k->P_/ ( k->P_+ k->R ); //gain;K(k) = P_(k)/( P_(k)+R );
			k->X = k->X_+ k->K * (k->z - k->X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
			k->P = (Trajectory(1,1,1) - k->K) * k->P_; //P(k) = (1-K(k))*P_(k);
		}
		//smoothed_trajectory.push_back(X);
		k->out_smoothed << i << " " << k->X.x << " " << k->X.y << " " << k->X.a << endl;
		//-
		// target - current
		double diff_x = k->X.x - k->x;//
		double diff_y = k->X.y - k->y;
		double diff_a = k->X.a - k->a;

		dx = dx + diff_x;
		dy = dy + diff_y;
		da = da + diff_a;

		//new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
		//
		k->out_new << i << " " << dx << " " << dy << " " << da << endl;        
    }

    if(p->mode == OPTICALFLOW_LK_2DOF){
        smth.at<double>(0,0) = 1; 
        smth.at<double>(0,1) = 0; 
        smth.at<double>(1,0) = 0; 
        smth.at<double>(1,1) = 1; 
    } else if (p->mode == OPTICALFLOW_LK_6DOF) {
        smth.at<double>(0,0) = cos(da);
        smth.at<double>(0,1) = -sin(da);
        smth.at<double>(1,0) = sin(da);
        smth.at<double>(1,1) = cos(da);
    }

    smth.at<double>(0,2) = dx;
    smth.at<double>(1,2) = dy;

    dl.Logger("calculate done dx %f dy %f", dx, dy);
    return ERR_NONE;
}

int Dove::CalculateMove_Integral(Mat& cur) {
    return ERR_NONE;
}

int Dove::CalculateMove_Tracker(Mat& cur) {
    return ERR_NONE;
}

int Dove::ApplyImage(Mat& src, bool scaled) {
    if( smth.at<double>(0,2) == 0.0 && smth.at<double>(1,2) == 0.0) {
        dl.Logger("no warp");
        return -1;
    }

    if(scaled == true) {
        smth.at<double>(0,2) = smth.at<double>(0,2) * p->scale;
        smth.at<double>(1,2) = smth.at<double>(1,2) * p->scale;      
    }

    warpAffine(src, src, smth, src.size());
    return ERR_NONE;
}

int Dove::Detect(Mat& cur) {
    vector<bbox_t>box;
    dt.Detect(cur, &box);
    return ERR_NONE;
}

int Dove::MakeMask() {
    mask = Mat::zeros(p->dst_height, p->dst_width, CV_8UC1);
    rectangle(mask, Point(p->sx, p->sy), Point(p->sx + p->width, p->sy + p->height), Scalar(255), -1);
    imwrite("saved/mask.png", mask);

    return ERR_NONE;
}
