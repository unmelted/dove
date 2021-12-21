
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
    tck.SetLogFilename("TEST");    
    _in = infile;
    _out = outfile;

    dl.Logger("Start construct. %s %s  ", _in.c_str(), _out.c_str());
    p->mode = mode;
    if(has_mask == true) 
        Initialize(true, coord);
    else 
        Initialize(false, 0);    
    tck.SetInitialData(p);      
}

Dove::Dove(string infile, string outfile) {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
    dt = Detection();
    tck = Tracking();    

    dl.SetLogFilename("TEST");
    dt.SetLogger(dl);
    tck.SetLogFilename("TEST");
    _in = infile;
    _out = outfile;
    Initialize(false, 0);    
    tck.SetInitialData(p);
}

void Dove::Initialize(bool has_mask, int* coord) {
    
    if(_in == "movie/4dmaker_600.mp4") {
        printf(" ------------ 600 !\n");        
        p->swipe_start = 80; //600 OK        
        p->swipe_end = 198;
    } else if (_in == "movie/4dmaker_603.mp4") {
        printf(" ------------ 603 !\n");
        p->swipe_start = 79;
        p->swipe_end = 181; //603 -- should conquer - couple gracking -- square merge needed 
    } else if (_in == "movie/4dmaker_626.mp4") {
        printf(" ------------ 626 !\n");        
        p->swipe_start = 79;
        p->swipe_end = 183; //626 OK emerald onepiece single
    } else if (_in == "movie/4dmaker_639.mp4") {
        p->swipe_start = 78;
        p->swipe_end = 130; //639 white shirts man single -- square merge needed
    } else if (_in == "movie/4dmaker_598.mp4") {
        p->swipe_start = 79;
        p->swipe_end = 165; //598 -- frame drop severe
    } else if (_in == "movie/4dmaker_607.mp4") {
        p->swipe_start = 79;
        p->swipe_end = 178; //silver onepiece single - missing during swipe because character
    }else if (_in == "movie/4dmaker_622.mp4") {
        p->swipe_start = 79;
        p->swipe_end = 165; //white onepiece single - missing during swipe because character
    }


    if (p->mode == OPTICALFLOW_LK_2DOF) {
        p->scale = 2;
        p->run_kalman = true;
        p->run_detection = false;

    } else if (p->mode == DARKNET_DETECT_MOVE) {
        p->scale = 2;
        p->run_kalman = true;
        p->run_detection = true;
        p->detector_type = DARKNET_YOLOV4;
    
    } else if (p->mode == BLOB_DETECT_TRACKING) {
        tck = Tracking();
        obj = new TRACK_OBJ();
        roi = new TRACK_OBJ();
        p->scale = 2;
        p->run_kalman = true;
        p->run_tracking = true;
        p->run_detection = false;        
        p->detector_type = BLOB_MSER;
        p->tracker_type = TRACKER_NONE; //tracker_none;
        p->track_scale = 3;
        p->limit_lx = 5;
        p->limit_ly = 5;
        p->limit_bx = 630;
        p->limit_by = 350;
        p->roi_w = 400;
        p->roi_h = 300;
        p->swipe_threshold = 15;
        p-> area_threshold = 200;
        p->iou_threshold = 0.3;
        p->center_threshold  = 60;
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
        p->smoothing_radius = 20;        
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
    out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(1930, 540));
    dl.Logger("Process TK started ");
    Mat src1oc; Mat src1o;
    int i = 0;
    int result = 0;
    int found = 0;
    vector <TransformParam> prev_to_cur_transform;
    TRACK_OBJ* pre_obj = new TRACK_OBJ();;

    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    
    int pframe = p->swipe_start - 5;

    while(true) {
        in >> src1oc;
        if(src1oc.data == NULL)
            break;
                
        if ( i == 0)
        {
            ImageProcess(src1oc, src1o);            
            SetRef(src1o);
            SetRefC(src1oc);            
            tck.SetBg(src1o, i);
            i++;
            continue;
        }

        if(i < pframe || i > p->swipe_end + 3) {
            i++;
            continue;            
        }

        ImageProcess(src1oc, src1o);
        // if( i == p->swipe_start)
        //     tck.PickArea(src1o, i, obj, roi);

        result = CalculateMove(src1o, i);
        replay_style = result;        

        //tck.DrawObjectTracking(src1o, obj, roi, false, replay_style);
        // sprintf(filename, "saved/%d_real.png", i);
        // imwrite(filename, src1o);
        // if ( i == p->swipe_start + 1)
        //      tck.SetBg(src1o);

        if (i > p->swipe_start && i <= p->swipe_end) {
//            tck.TrackerUpdate(src1o, i, obj, roi);            
            double dx = 0;
            double dy = 0;
            double da = 0;
            if(!tck.issame) {
                dx = (pre_obj->cx - obj->cx) * p->track_scale;
                dy = (pre_obj->cy - obj->cy) * p->track_scale;
            }
            k->out_transform << (i - p->swipe_start - 1) << " "<< dx << " "<< dy << " " << da << endl;            
            prev_to_cur_transform.push_back(TransformParam(dx, dy, 0));
        } else {
            prev_to_cur_transform.push_back(TransformParam(0, 0, 0));
        }

        if (tck.isfound) {
            obj->copy(pre_obj);
        }

        SetRef(src1o);
        SetRefC(src1oc);
        i++;
        // if(i == 50)
        //      break;

        dl.Logger("[%d] Image Analysis  %f ", i, LapTimer(all));        
    }

    //return ERR_NONE;	
    dl.Logger("PostPrcess start ... ");
    double a = 0;
    double x = 0;
    double y = 0;
    vector <Trajectory> trajectory; // trajectory at all frames

    for(size_t i = 0; i < prev_to_cur_transform.size(); i++) {
        x += prev_to_cur_transform[i].dx;
        y += prev_to_cur_transform[i].dy;
        a += prev_to_cur_transform[i].da;

        trajectory.push_back(Trajectory(x,y,a));
        k->out_trajectory << (i+1) << " " << x << " " << y << " " << a << endl;
    }

    // Step 3 - Smooth out the trajectory using an averaging window
    vector <Trajectory> smoothed_trajectory; // trajectory at all frames
    for(size_t i = 0; i < trajectory.size(); i++) {
        double sum_x = 0;
        double sum_y = 0;
        double sum_a = 0;
        int count = 0;

        for(int j = -p->smoothing_radius; j <= p->smoothing_radius; j++) {
            if(i+j >= 0 && i+j < trajectory.size()) {
                sum_x += trajectory[i+j].x;
                sum_y += trajectory[i+j].y;
                sum_a += trajectory[i+j].a;

                count++;
            }
        }

        double avg_a = sum_a / count;
        double avg_x = sum_x / count;
        double avg_y = sum_y / count;

        smoothed_trajectory.push_back(Trajectory(avg_x, avg_y, avg_a));
        k->out_smoothed << (i+1) << " " << avg_x << " " << avg_y << " " << avg_a << endl;
    }

    // Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
    vector <TransformParam> new_prev_to_cur_transform;
    // Accumulated frame to frame transform
    a = 0;
    x = 0;
    y = 0;

    for(size_t i = 0; i < prev_to_cur_transform.size(); i++) {
        x += prev_to_cur_transform[i].dx;
        y += prev_to_cur_transform[i].dy;
        a += prev_to_cur_transform[i].da;

        // target - current
        double diff_x = smoothed_trajectory[i].x - x;
        double diff_y = smoothed_trajectory[i].y - y;
        double diff_a = smoothed_trajectory[i].a - a;

        double dx = prev_to_cur_transform[i].dx + diff_x;
        double dy = prev_to_cur_transform[i].dy + diff_y;
        double da = prev_to_cur_transform[i].da + diff_a;

        new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
        k->out_new << (i+1) << " " << dx << " " << dy << " " << da << endl;
    }

    int k = 0;
    i = 0;
    VideoCapture in2(_in);    
    while(true) {
        in2 >> src1oc;
        if(src1oc.data == NULL)
            break;

        if(src1oc.cols > p->dst_width)
            resize(src1oc, src1oc, Size(p->dst_width, p->dst_height));

        if ( i == 0)
        {
            SetRefC(src1oc);
            i++;
            continue;
        }
        //sprintf(filename, "saved/%d_apply.png", i);
        Mat canvas = Mat::zeros(540, 1930, CV_8UC3);
        if (i >= p->swipe_start && i <= p->swipe_end) {
            smth.at<double>(0,0) = 1; 
            smth.at<double>(0,1) = 0; 
            smth.at<double>(1,0) = 0; 
            smth.at<double>(1,1) = 1; 
            smth.at<double>(0,2) = -new_prev_to_cur_transform[k].dx;
            smth.at<double>(1,2) = -new_prev_to_cur_transform[k].dy;        
            k++;
            dl.Logger("will Apply %f %f ", smth.at<double>(0,2), smth.at<double>(1,2));
            ApplyImageRef();
            // imwrite(filename, refcw);
        }
        else {
            refc.copyTo(refcw);
        }
        resize(refc, refc, Size(960, 540));        
        resize(refcw, refcw, Size(960, 540));
        refc.copyTo(canvas(Range::all(), Range(0, 960)));
        refcw.copyTo(canvas(Range::all(), Range(970, 1930)));

        out << canvas;
        SetRefC(src1oc);
        i++;
    }

    dl.Logger(".. %f", LapTimer(all));
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
    //if tk on? 
    if(!p->run_tracking)
        GaussianBlur(temp, dst, {p->blur_size, p->blur_size}, p->blur_sigma, p->blur_sigma);
    else
        temp.copyTo(dst);
    
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
        float fret = 0.0;
        fret = tck.DetectAndTrack(cur, frame_id, obj, roi);
        dl.Logger("[%d] result %f isFound %d issmae %d ", frame_id, fret, tck.isfound, tck.issame);

        if (tck.isfound == true) {
            if( fret >= p->swipe_threshold * tck.first_summ && swipe_on == false) {
                swipe_on = true;
                result = SWIPE_ON;            
                dl.Logger("[%d] SWIPE START ", frame_id);
            }
            else if (swipe_on == true) {
                result = KEEP_TRACKING_SWIPE;
                dl.Logger("[%d] KEEP TRACKING SWIPE", frame_id); 
            }
            else {
                result = KEEP_TRACKING;
                dl.Logger("[%d] KEEP TRACKING", frame_id); 
            }
            if (tck.issame == true) {
                dl.Logger("[%d] SAME ", frame_id);      
            }

        }    
        else if (tck.isfound == false){ 
            result = TRACK_NONE;
            dl.Logger("[%d] NONE", frame_id);             
        }
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
