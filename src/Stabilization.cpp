
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

Dove::Dove(int mode, bool has_mask, int* coord, string infile, string outfile, string id) {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
    dt = Detection();
    tck = Tracking();

    dl.Logger("instance created.. ");
    dl.SetLogFilename("TEST");
    dt.SetLogger(dl);    
    tck.SetParam(p);
    tck.SetLogger(dl);    
    _in = infile;
    _out = outfile;

    dl.Logger("Start construct. %s %s  ", _in.c_str(), _out.c_str());
    p->mode = mode;
    if(has_mask == true) 
        Initialize(true, coord);
    else 
        Initialize(false, 0);    
}

Dove::Dove(string infile, string outfile) {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
    dt = Detection();
    tck = Tracking();    

    dl.SetLogFilename("TEST");
    dt.SetLogger(dl);
    tck.SetParam(p);
    tck.SetLogger(dl);    
    _in = infile;
    _out = outfile;
    Initialize(false, 0);    

}

void Dove::Initialize(bool has_mask, int* coord) {
    
    if (p->mode == OPTICALFLOW_LK_2DOF) {
        p->scale = 2;
        p->run_kalman = true;
        p->run_detection = false;
        p->swipe_start = 80;
        p->swipe_end = 198;

    } else if (p->mode == DARKNET_DETECT_MOVE) {
        p->scale = 2;
        p->run_kalman = true;
        p->run_detection = true;
    } else if (p->mode == BLOB_DETECT_TRACKING) {
        tck = Tracking();
        p->scale = 2;
        p->run_kalman = true;
        p->run_tracking = true;
        p->detector_type = BLOB_MSER;
        p->track_scale = 5;
        p->limit_lx = 5;
        p->limit_ly = 5;
        p->limit_bx = 630;
        p->limit_by = 470;
        p->roi_w = 400;
        p->roi_h = 300;
        p-> area_threshold = 200;
        p->iou_threshold = 0.3;
        p->center_threshold  = 30;
        obj = NULL;
        roi = NULL;        
    }

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
        if(p->detector_type == DARKNET_YOLOV4) {
            dt = Detection();
            p->detect_threshold = 0.5;
            p->detector_type = DARKNET_YOLOV4;
            p->names_file = "darknet/data/coco.names";
            p->cfg_file = "darknet/cfg/yolov4-tiny.cfg";
            p->weights_file = "darknet/weights/yolov4-tiny.weights";
            p->id_filter.push_back(0); //id based on coco names            
            obj_trajectory.open("analysis/detected_obj.txt");
            obj_c_trajectory.open("analysis/detected_obj_center.txt");        
            dt.LoadModel(p);            
        }
    }

    if(p->run_kalman == true) {
        k = new KALMAN();
        k->Q.set(k->pstd, k->pstd, k->pstd);
        k->R.set(k->cstd, k->cstd, k->cstd);      
        k->out_transform.open("analysis/prev_to_cur_transformation.txt");
        k->out_trajectory.open("analysis/trajectory.txt");
        k->out_smoothed.open("analysis/smoothed_trajectory.txt");
        k->out_new.open("analysis/new_prev_to_cur_transformation.txt");
    }

    smth.create(2 , 3 , CV_64F);        

    dl.Logger("Initialized compelete.");    
}

Dove::~Dove() {
    if(k != NULL && p->run_kalman)
         delete k;
    dl.Logger("delete k ");                 
    delete p;
    delete t;
    dl.Logger("delete p ,t ");
}

int Dove::Process() {
    if(p->mode == OPTICALFLOW_LK_2DOF)
        ProcessLK();
    else if (p->mode == BLOB_DETECT_TRACKING)
        ProcessTK();

    return ERR_NONE;
}   

int Dove::ProcessTK() {
    VideoCapture in(_in);
    VideoWriter out;    
    out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(p->dst_width, p->dst_height));

    Mat src1oc; Mat src1o;
    int i = 0;
    int result = 0;
    int found = 0;
	
    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    

    while(true) {
        in >> src1oc;
        if(src1oc.data == NULL)
            break;
        
        ImageProcess(src1oc, src1o);

        if ( i == 0)
        {
            SetRef(src1o);
            SetRefC(src1oc);            
            tck.SetBg(src1oc);
            i++;
            continue;
        }
            
        result = CalculateMove(src1o, i);
        if(result != MISSED_TRACKING) {
            p->replay_style = result;
            tck.DrawObjectTracking(src1oc, obj, roi, result);
        }

        imshow("FIRST PROCESS", src1oc);
        SetRef(src1o);
        SetRefC(src1oc);
        i++;
        // if(i == 50)
        //      break;
    }
    Logger("Image Analysis  %f ", LapTimer(all));		
    //post process
    dl.Logger("PostPrcess start ... ");
    while(true) {
        in >> src1oc;
        if(src1oc.data == NULL)
            break;

        //ApplyImage();        
        if(refcw.cols > p->dst_width)
            resize(refcw, refcw, Size(p->dst_width, p->dst_height));

        sprintf(filename, "saved/%d_.png", i);
        imwrite(filename, refcw);

        out << refcw;        
        dl.Logger(".. %f", LapTimer(all));
    }

    return ERR_NONE;

}

int Dove::ProcessLK() {
    dl.Logger(" LK process start..");

    VideoCapture in(_in);
    VideoWriter out;    
    out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(p->dst_width, p->dst_height));

    Mat src1oc; Mat src1o;
    int i = 0;
    int result = 0;
    int found = 0;

    while(true) {
        in >> src1oc;
        if(src1oc.data == NULL)
            break;
        
        ImageProcess(src1oc, src1o);

        if ( i == 0)
        {
            if(p->mode == OPTICALFLOW_LK_2DOF || p->mode == OPTICALFLOW_LK_6DOF) {            
                SetRef(src1o);
                SetRefC(src1oc);            
            }
            i++;
            continue;
        }
            
        result = CalculateMove(src1o, i);
        ApplyImageRef();        
        if(refcw.cols > p->dst_width)
            resize(refcw, refcw, Size(p->dst_width, p->dst_height));

        out << refcw;        
        sprintf(filename, "saved/%d_.png", i);
        imwrite(filename, refcw);

        SetRef(src1o);
        SetRefC(src1oc);
        i++;
        // if(i == 50)
        //      break;
    }

    return ERR_NONE;
};

int Dove::ImageProcess(Mat& src, Mat& dst) {
    Mat temp;
    if(p->scale != 1)
        resize(src, temp, Size(int((float)src.cols/p->scale), int(float(src.rows)/p->scale)), 0,0,1);
    else 
        src.copyTo(temp);

    cvtColor(temp, temp, COLOR_BGR2GRAY);
    GaussianBlur(temp, dst, {p->blur_size, p->blur_size}, p->blur_sigma, p->blur_sigma);

    if(p->has_mask)
        MakeMask();

    return ERR_NONE;
}

int Dove::CalculateMove(int frame_id) {

    smth.at<double>(0,0) = 1; 
    smth.at<double>(0,1) = 0; 
    smth.at<double>(1,0) = 0; 
    smth.at<double>(1,1) = 1; 
    smth.at<double>(0,2) = -dt_comp[frame_id -1].dx;
    smth.at<double>(1,2) = -dt_comp[frame_id -1].dy;

    return ERR_NONE;
}

int Dove::CalculateMove(Mat& cur, int frame_id) {
    int result = -1;
    if(p->mode == OPTICALFLOW_LK_2DOF || p->mode == OPTICALFLOW_LK_6DOF) {
        result = CalculateMove_LK(cur, frame_id);
        
    } else if (p->mode == INTEGRAL_IMAGE) {
        result = CalculateMove_Integral(cur);

    } else if (p->mode == BLOB_DETECT_TRACKING) {
        result = tck.DetectAndTrack(cur, frame_id, obj, roi);
        if (tck.isfound == true && obj != NULL) {
            if( result >= p->swipe_threshold * tck.first_summ && swipe_on == false) {
                swipe_on = true;
                result = SWIPE_START;            
            } else if (tck.issame == true && swipe_on == true) {
                result = SWIPE_END;
                swipe_on = false;
            } else if (tck.issame == true && swipe_on == false)
                result = PAUSE_PERIOD;
            else             
                result = KEEP_TRACKING;
        }    
        else if (tck.isfound == false && obj == NULL)
            result = MISSED_TRACKING;
    } else if( p->mode == DARKNET_DETECT_MOVE) {
        Detect(cur, frame_id);
        result = CalculateMove(frame_id);
        if(objects[i].obj_cnt > 0 ) {
             dt.DrawBoxes(refcw, objects[i - 1].bbx);
        }
    } 
    return result;
}

int Dove::Detect(Mat cur, int frame_id) {
    int result = -1;
    vector<bbox_t>box;
    Mat dtin;
    cur.copyTo(dtin);
    result = dt.Detect(dtin, &box);
    dl.Logger("[%d] detect result %d cnt [%d] ", frame_id, result, box.size());    
    if(result < ERR_NONE)
        return result;
    
    if(box.size() > 0) {
        dt.ShowResult(box, frame_id);
        DT_OBJECTS n(frame_id, box.size(), box);
        n.calCenter();
        objects.insert({frame_id, n});
        for(int i = 0; i < n.obj_cnt; i ++) {
            dl.Logger("[%d] %d %d %d %d  - cx %d cy %d", frame_id, n.bbx[i].x , n.bbx[i].y,  n.bbx[i].w ,n.bbx[i].h, n.cx[i], n.cy[i]);
            obj_trajectory << frame_id << " " << n.bbx[i].x << " " << n.bbx[i].y << " " << n.bbx[i].w << " " << n.bbx[i].h << endl;
            obj_c_trajectory << frame_id << " " << n.cx[i] << " " << n.cy[i] << endl;
        }
        dl.Logger("objects size %d -- cx %d ", objects.size(), objects[frame_id].cx[0]);
        if( frame_id >= 2 && objects[frame_id -1].obj_cnt > 0) {
            DT_XY m;
            dl.Logger("Insert 1 cx %d cx-1 %d cy %d cy-1 %d ", objects[frame_id].cx[0], objects[frame_id -1].cx[0],
                    objects[frame_id].cy[0], objects[frame_id -1].cy[0]);   

            m.dx = objects[frame_id].cx[0] - objects[frame_id -1].cx[0];
            m.dy = objects[frame_id].cy[0] - objects[frame_id -1].cy[0];
            dl.Logger("Insert DT_XY %d %d", m.dx, m.dy);
            dt_comp.insert({frame_id, m});
        }
        else {
            DT_XY m;
            m.dx = 0;
            m.dy = 0;    
            dt_comp.insert({frame_id, m});
        }
    }
    else {
        DT_OBJECTS n(frame_id);
        objects.insert({frame_id , n});
        obj_trajectory << frame_id << " 0 0 "<< endl;                    
        obj_c_trajectory << frame_id << " 0 0 " << endl;        
        DT_XY m;
        m.dx = 0;
        m.dy = 0;    
        dt_comp.insert({frame_id, m});
    }
    
    
    return ERR_NONE;
}
int Dove::CalculateMove_LK(Mat& cur, int frame_id) {
    static int i = 1;
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
        goodFeaturesToTrack(ref, features1, 200, 0.01, 30, mask);
    else 
        goodFeaturesToTrack(ref, features1, 200, 0.01, 30);    
    calcOpticalFlowPyrLK(ref, cur, features1, features2, status, err );

    for(size_t i = 0; i < status.size(); i++)
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
    }

    if(affine.empty() == true) {
        dl.Logger("there is no solution ..");
        pre_affine.copyTo(affine);
    }

    double dx = affine.at<double>(0,2);
    double dy = affine.at<double>(1,2);
    if( p->run_detection == true &&
        abs(dt_comp[frame_id - 1].dx) > 0 && abs(dt_comp[frame_id - 1].dx) < 20 &&
        abs(dt_comp[frame_id - 1].dy) > 0 && abs(dt_comp[frame_id - 1].dy) < 20)    
    {
        dx +=  dt_comp[frame_id - 1].dx;
        dy +=  dt_comp[frame_id - 1].dy;
    }

    double da = atan2(affine.at<double>(1,0), affine.at<double>(0,0));
    double ds_x = affine.at<double>(0,0)/cos(da);
    double ds_y = affine.at<double>(1,1)/cos(da);

    dl.Logger("origin dx %f dy %f", dx ,dy);

    if(p->run_kalman) {
        k->out_transform << i << " " << dx << " " << dy << " " << da << endl;        
		k->x += dx;
		k->y += dy;
		k->a += da;
		//trajectory.push_back(Trajectory(x,y,a));
		//
		k->out_trajectory << i << " " << k->x << " " << k->y << " " << k->a << endl;
		k->z = Trajectory(k->x, k->y, k->a);

		if(i == 1){
			k->X = Trajectory(0,0,0); //Initial estimate,  set 0
			k->P = Trajectory(1,1,1); //set error variance,set 1
		}
		else
		{
			//time update（prediction）
			k->X_ = k->X; //X_(k) = X(k-1);
			k->P_ = k->P+ k->Q; //P_(k) = P(k-1)+Q;
			// measurement update（correction）
			k->K = k->P_/ ( k->P_+ k->R ); //gain;K(k) = P_(k)/( P_(k)+R );
			k->X = k->X_+ k->K * (k->z - k->X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
			k->P = (Trajectory(1,1,1) - k->K) * k->P_; //P(k) = (1-K(k))*P_(k);
		}
		//smoothed_trajectory.push_back(X);
		k->out_smoothed << i << " " << k->X.x << " " << k->X.y << " " << k->X.a << endl;

		// target - current
		double diff_x = k->X.x - k->x;//
		double diff_y = k->X.y - k->y;
		double diff_a = k->X.a - k->a;

		dx = dx + diff_x;
		dy = dy + diff_y;
		da = da + diff_a;
        dl.Logger("from kalman %f %f ", dx, dy);
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
    i++;

    //dl.Logger("calculate done dx %f dy %f", dx, dy);
    return ERR_NONE;
}

int Dove::CalculateMove_Integral(Mat& cur) {
    return ERR_NONE;
}

int Dove::CalculateMove_Tracker(Mat& cur) {
    return ERR_NONE;
}

void Dove::ApplyImage(Mat& src, bool scaled) {
    if( smth.at<double>(0,2) == 0.0 && smth.at<double>(1,2) == 0.0) {
        dl.Logger("no warp");
        return;
    }

    if(scaled == true) {
        smth.at<double>(0,2) = smth.at<double>(0,2) * p->scale;
        smth.at<double>(1,2) = smth.at<double>(1,2) * p->scale;      
    }

    dl.Logger("apply image %f %f ", smth.at<double>(0,2), smth.at<double>(1,2));
    warpAffine(src, src, smth, src.size());
    // if(scaled == true) {
    //      static int i = 1;
    //      sprintf(filename, "saved/%d_apply.png", i);
    //      imwrite(filename, src);
    //      i++;
    // }
}

void Dove::ApplyImageRef() {
    warpAffine(refc, refcw, smth, refc.size());
}

int Dove::MakeMask() {
    mask = Mat::zeros(p->dst_height, p->dst_width, CV_8UC1);
    mask.setTo(Scalar(255));
    rectangle(mask, Point(p->sx, p->sy), Point(p->sx + p->width, p->sy + p->height), Scalar(0), -1);
    imwrite("saved/mask.png", mask);

    return ERR_NONE;
}
