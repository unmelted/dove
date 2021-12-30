
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
using namespace dove;

Dove::Dove(int event, bool has_mask, int* coord, string infile, string outfile, string id) {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
#if defined _MAC_
    dt = Detection();
#endif
    dl.SetLogFilename("TEST");    
    dl.Logger("instance created.. ");

#if defined _MAC_
    dt.SetLogger(dl);    
#endif
    _in = infile;
    _out = outfile;

    dl.Logger("Start construct. %s %s  ", _in.c_str(), _out.c_str());
    p->event = event;
    Initialize(false, coord);
}

Dove::Dove(string infile, string outfile) {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
#if defined _MAC_
    dt = Detection();
#endif
    dl.SetLogFilename("TEST");

#if defined _MAC_
    dt.SetLogger(dl);
#endif
    _in = infile;
    _out = outfile;
    Initialize(false, 0);    
}

void Dove::Initialize(bool has_mask, int* coord) {
#if defined _MAC_
    if(_in == "movie/4dmaker_600.mp4" || _in == "movie/4dmaker_600_out2.mp4") {
#else
    if (_in == "movie\\4dmaker_600.mp4" || _in == "movie\\4dmaker_600_out2.mp4") {
#endif
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
    } else if (_in == "movie/4NylanderGoal.mp4") {
        p->swipe_start = 153;
        p->swipe_end = 188; 
    } else if (_in == "movie/BUT_TATAR_4-0.mp4") {
        p->swipe_start = 188;
        p->swipe_end = 232; 
    } else if (_in == "movie/2018_02_09_17_58_50.mp4") {
        p->swipe_start = 64;
        p->swipe_end = 88; //short track
        p->roi_sx = 960;
        p->roi_sy = 540;
    } else if (_in == "movie/2018_02_25_09_55_28.mp4") {
        p->swipe_start = 58;
        p->swipe_end = 91; //short track
        p->roi_sx = 1160; 
        p->roi_sy = 730;
    } else if (_in == "movie/2018_02_13_19_37_53_0.mp4") {
        p->swipe_start = 51;
        p->swipe_end = 84; //short track
        p->roi_sx = 430; 
        p->roi_sy = 850;
    }

    if (p->event == FIGURE) {
        p->colored = false;    
        p->mode = DETECT_TRACKING;
        p->roi_input = false;
    } else if(p->event == HOCKEY || p->event == SHORT) {
        p->colored = true;
        p->mode = DETECT_TRACKING;        
        p->roi_input = true;
        p->roi_sx = 430;
        p->roi_sy = 850;
    } 
    p->scale = 2;

    if (p->mode == OPTICALFLOW_LK_2DOF) {
        p->scale = 2;
        p->run_kalman = true;
        p->run_detection = false;

    } else if (p->mode == DARKNET_DETECT_MOVE) {
        p->scale = 2;
        p->run_kalman = true;
        p->run_detection = true;
        p->detector_type = DARKNET_YOLOV4;
    
    } else if (p->mode == DETECT_TRACKING || 
            p->mode == DETECT_TRACKING_CH) {
        obj = new TRACK_OBJ();
        roi = new TRACK_OBJ();
        p->run_tracking =   true;
        p->run_detection = false;        
        p->detector_type = BLOB_MSER;
        p->tracker_type = CSRT; //tracker_none;
        p->track_scale = 3;
        p->limit_lx = 5;
        p->limit_ly = 5;
        p->limit_bx = 630;
        p->limit_by = 350;
        p->roi_w = 200;
        p->roi_h = 160;
        p->swipe_threshold = 15;
        p->area_threshold = 200;
        p->iou_threshold = 0.3;
        p->center_threshold  = 60;

        p->run_path_smoothing = true;
        p->smoothing_radius = 30;
        p->run_kalman = false;
        p->run_kalman_pre = false;
        p->run_kalman_post = false;        
    }

    if(p->colored == false) {
        p->colored = false;
        tck = new GrayTracking();    
        tck->SetLogFilename("TEST");
        tck->SetInitialData(p);
    } else if(p->colored == true) {
        p->colored = true;
        tck = new ColoredTracking();    
        tck->SetLogFilename("TEST");
        tck->SetInitialData(p);
    }

    //if(has_mask == true) 
    // {
        //p->has_mask = true;
        // p->sx = coord[0] / p->scale;
        // p->sy = coord[1] / p->scale;
        // p->width = coord[2] / p->scale;
        // p->height = coord[3] / p->scale;    
    // }

    p->blur_size = 5;
    p->blur_sigma = 0.7;
    p->dst_width = 1920;
    p->dst_height = 1080;

    if(p->run_detection == true) {
#if defined _MAC_
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
#endif
    }

    k = new KALMAN();
    k->pstd = (double)coord[0]/10000;
    k->cstd = (double)coord[1]/10000;
    //dl.Logger("pstd %f cstd %f ", k->pstd, k->cstd);

    if(p->run_path_smoothing == true || p->run_kalman == true)  {
        k->out_transform.open("analysis/prev_to_cur_transformation.txt");
        k->out_trajectory.open("analysis/trajectory.txt");
        k->out_smoothed.open("analysis/smoothed_trajectory.txt");
        k->out_new.open("analysis/new_prev_to_cur_transformation.txt");        
    }

    if(p->run_kalman == true || 
        p->run_kalman_pre == true || p->run_kalman_post == true) {    
        k->Q.set(k->pstd, k->pstd, k->pstd);
        k->R.set(k->cstd, k->cstd, k->cstd);      
        k->out_transform2.open("analysis/prev_to_cur_transformation2.txt");
        k->out_trajectory2.open("analysis/trajectory2.txt");
        k->out_smoothed2.open("analysis/smoothed_trajectory2.txt");
        k->out_new2.open("analysis/new_prev_to_cur_transformation2.txt");        
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
    else if (p->mode == DETECT_TRACKING)
        ProcessTK();
    else if (p->mode == DETECT_TRACKING_CH)
        ProcessChristmas();

    return ERR_NONE;
}   

int Dove::ProcessTK() {
    bool compare = false;
#if defined GPU
    // cv::Ptr<cudacodec::VideoReader> in = cudacodec::createVideoReader(_in);
    // cv::Ptr<cudacodec::VideoWriter> out = cudacodec::createVideoWriter(_out, Size(p->dst_width, p->dst_height), 30);
    VideoCapture in(_in);
    VideoWriter out;
    if (compare)
        out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(1930, 540));
    else
        out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(p->dst_width, p->dst_height));

#else
    VideoCapture in(_in);
    VideoWriter out;
    if (compare)
        out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(1930, 540));
    else
        out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(p->dst_width, p->dst_height));
#endif

    dl.Logger("Process TK started ");
#if defined GPU
    cuda::GpuMat src1ocg; 
    cuda::GpuMat src1og;
#endif

    Mat src1oc; Mat src1o;
    int i = 0;
    int result = 0;
    int found = 0;
    vector <TransformParam> prev_to_cur_transform;
    TRACK_OBJ* pre_obj = new TRACK_OBJ();;

    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    
    int t_frame_start = p->swipe_start;
    int t_frame_end = p->swipe_end;

    bool oversampling = false;
    vector<int>same;

    while(true) {
#if defined GPU
        // if (!in->nextFrame(src1ocg))
        //     break;
        // ImageProcess(src1ocg, src1og);
        in >> src1oc;
        if(src1oc.data == NULL)
            break;
        src1ocg.upload(src1oc);
        ImageProcess(src1ocg, src1og);

#else 
        in >> src1oc;
        if(src1oc.data == NULL)
            break;
        ImageProcess(src1oc, src1o);
#endif

        if ( i == 0)
        {                     
#if defined GPU
            tck->SetBg(src1og, i);
#else
            tck->SetBg(src1o, i);
#endif
            i++;
            continue;
        }

        if (i > t_frame_end)
            break;

        if(i < t_frame_start || i > t_frame_end) {         
            i++;
            continue;            
        }
        printf("OK ? %d \n", i);
        if( p->tracker_type != TRACKER_NONE) {
            if (i == t_frame_start)
#if defined GPU
                if (p->roi_input)
                    tck->TrackerInitFx(src1og, i, p->roi_sx, p->roi_sy, obj, roi);
                else
                    tck->TrackerInit(src1og, i, obj, roi);
            else
                tck->TrackerUpdate(src1og, i, obj, roi);
#else
                if(p->roi_input)
                    tck->TrackerInitFx(src1o, i, p->roi_sx, p->roi_sy, obj, roi);                
                else 
                    tck->TrackerInit(src1o, i, obj, roi);
            else
                tck->TrackerUpdate(src1o, i, obj, roi);     
#endif
        } else {
            result = CalculateMove(src1o, i);
            replay_style = result;        

        }
        //tck->DrawObjectTracking(src1o, obj, roi, false, replay_style);
        double dx = 0;
        double dy = 0;
        double da = 0;
        if (i > t_frame_start && i <= t_frame_end) {

            if(!tck->issame) 
            { 
                dx = (pre_obj->cx - obj->cx) * p->track_scale;
                dy = (pre_obj->cy - obj->cy) * p->track_scale;
                dl.Logger("pre origin %f %f ", dx, dy);
                if(p->run_kalman || p->run_kalman_pre) {
                    k->x += dx;
                    k->y += dy;
                    k->a += da;
                    k->out_transform2 << i << " " << dx << " " << dy << " " << da << endl;

                    k->z = dove::Trajectory(k->x, k->y, k->a);
                    if( i == p->swipe_start ){
                        k->X = dove::Trajectory(0,0,0); //Initial estimate,  set 0
                        k->P = dove::Trajectory(1,1,1); //set error variance,set 1
                    }
                    else
                    {
                        //time update（prediction）
                        k->X_ = k->X; //X_(k) = X(k-1);
                        k->P_ = k->P+ k->Q; //P_(k) = P(k-1)+Q;
                        // measurement update（correction）
                        k->K = k->P_/ ( k->P_+ k->R ); //gain;K(k) = P_(k)/( P_(k)+R );
                        k->X = k->X_+ k->K * (k->z - k->X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
                        k->P = (dove::Trajectory(1,1,1) - k->K) * k->P_; //P(k) = (1-K(k))*P_(k);
                    }
                    //smoothed_trajectory.push_back(X);
                    k->out_smoothed2 << i << " " << k->X.x << " " << k->X.y << " " << k->X.a << endl;

                    // target - current
                    double diff_x = k->X.x - k->x;//
                    double diff_y = k->X.y - k->y;
                    double diff_a = k->X.a - k->a;

                    dx = dx + diff_x;
                    dy = dy + diff_y;
                    da = da + diff_a;
                    dl.Logger("pre from kalman %f %f ", dx, dy);
                    //new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
                    //
                    k->out_new2 << i << " " << dx << " " << dy << " " << da << endl;     
                    prev_to_cur_transform.push_back(TransformParam(dx, dy, 0));                    
                } 
                else if (p->run_path_smoothing == true) {      
                    printf("[%d] not same. out_transform without kalman", i);                    
                    k->out_transform << i << " "<< dx << " "<< dy << " " << da << endl;            
                    prev_to_cur_transform.push_back(TransformParam(dx, dy, 0));
                } else {
                    k->out_transform << i << " "<< dx << " "<< dy << " " << da << endl;            
                    prev_to_cur_transform.push_back(TransformParam(dx, dy, 0));                    
                }
            } else {
                same.push_back(i);
                printf("[%d] same. out_transform without kalman", i);
                k->out_transform << i << " "<< dx << " "<< dy << " " << da << endl;            
                prev_to_cur_transform.push_back(TransformParam(dx, dy, 0));
            }
        }

        if (tck->isfound) {
            obj->copy(pre_obj);
        }
        i++;

        //direct apply
        // {
        //     smth.at<double>(0,0) = 1; 
        //     smth.at<double>(0,1) = 0; 
        //     smth.at<double>(1,0) = 0; 
        //     smth.at<double>(1,1) = 1;         
        //     smth.at<double>(0,2) = -dx;
        //     smth.at<double>(1,2) = -dy;
        //     resize(refc, refc, Size(1920, 1080));               
        //     ApplyImageRef();    
        //     printf("refcw apply %f %f \n", smth.at<double>(0,2), smth.at<double>(1,2));
        // }
        // out << refcw;
        // if(i == 50)
        //      break;

    }

    dl.Logger("[%d] Image Analysis  %f ", i, LapTimer(all));        
    //return ERR_NONE;	

    dl.Logger("PostPrcess start ... ");
    double a = 0;
    double x = 0;
    double y = 0;
    vector <dove::Trajectory> trajectory; // trajectory at all frames

    for(size_t i = 0; i < prev_to_cur_transform.size(); i++) {
        if(oversampling ) {
            if(prev_to_cur_transform[i+1].dx == 0 && prev_to_cur_transform[i+1].dy == 0) {
                x += prev_to_cur_transform[i].dx/2;
                y += prev_to_cur_transform[i].dy/2;
                a += 0;

                trajectory.push_back(dove::Trajectory(x,y,a));
                k->out_trajectory << (i) << " " << x << " " << y << " " << a << endl;
                x += prev_to_cur_transform[i].dx/2;
                y += prev_to_cur_transform[i].dy/2;
                a += 0;

                trajectory.push_back(dove::Trajectory(x,y,a));
                k->out_trajectory << (i) << " " << x << " " << y << " " << a << endl;
            }

        } else {
            x += prev_to_cur_transform[i].dx;
            y += prev_to_cur_transform[i].dy;
            a += prev_to_cur_transform[i].da;

            trajectory.push_back(dove::Trajectory(x,y,a));
            k->out_trajectory << (i+1) << " " << x << " " << y << " " << a << endl;
        }
    }

    // Step 3 - Smooth out the trajectory using an averaging window
    vector <dove::Trajectory> smoothed_trajectory; // trajectory at all frames
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

        smoothed_trajectory.push_back(dove::Trajectory(avg_x, avg_y, avg_a));
        k->out_smoothed << (i+1) << " " << avg_x << " " << avg_y << " " << avg_a << endl;
    }

    // Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
    vector <TransformParam> new_prev_to_cur_transform;
    // Accumulated frame to frame transform
    a = 0;
    x = 0;
    y = 0;
    double minx = 2000;
    double maxx = 0;
    double miny = 2000;
    double maxy = 0;

    for(size_t i = 0; i < smoothed_trajectory.size(); i++) {
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
        if(dx < minx)
            minx = dx;
        else if(dx > maxx)
            maxx = dx;
        if(dy < miny)
            miny = dy;
        else if (dy > maxy)
            maxy = dy;
    }
    dl.Logger("minx %f maxx %f miny %f maxy %f", minx, maxx, miny, maxy);
    Rect mg;
    CalculcateMargin(minx, maxx, miny, maxy, &mg);

    int vi = 0;
    int m = 0;    
    i = 0; 
    char tx[10];    
#if defined GPU
    // Ptr<cudacodec::VideoReader> in2 = cudacodec::createVideoReader(_in);
    VideoCapture in2(_in);        
#else
    VideoCapture in2(_in);    
#endif
    while(true) {
#if defined GPU
        // if (!in2->nextFrame(src1ocg))
        //     break;
        // ImageProcess(src1ocg, src1og);
        in >> src1oc;
        if(src1oc.data == NULL)
            break;
        src1ocg.upload(src1oc);
        ImageProcess(src1ocg, src1og);

#else 
        in2 >> src1oc;
        if(src1oc.data == NULL)
            break;
        ImageProcess(src1oc, src1o);
#endif

        if(src1oc.cols > p->dst_width)
            cv::resize(src1oc, src1oc, Size(p->dst_width, p->dst_height));

        if (i == 0)
        {
#if defined GPU
            SetRefCG(src1ocg);
#else
            SetRefC(src1oc);
#endif
            i++;
            continue;
        }
        //sprintf(filename, "saved/%d_apply.png", i);
#if defined GPU
        cuda::GpuMat canvas;
        if (compare)
            canvas = cuda::GpuMat(540, 1930, CV_8UC3);
        else
            canvas = cuda::GpuMat(1080, 1920, CV_8UC3);
#else
        Mat canvas;
        if (compare)
            canvas = Mat::zeros(540, 1930, CV_8UC3);
        else
            canvas = Mat::zeros(1080, 1920, CV_8UC3);
#endif

        if (i > t_frame_start && i <= t_frame_end) {
            smth.at<double>(0,0) = 1; 
            smth.at<double>(0,1) = 0; 
            smth.at<double>(1,0) = 0; 
            smth.at<double>(1,1) = 1; 
            double dx = -new_prev_to_cur_transform[vi].dx;
            double dy = -new_prev_to_cur_transform[vi].dy;

            if(p->run_kalman_post) {
                k->x += dx;
                k->y += dy;
                k->a += 0;
                dl.Logger("post origin %f %f ", dx, dy);                
                k->out_transform << i << " " << dx << " " << dy << " " << 0 << endl;

                k->z = dove::Trajectory(k->x, k->y, k->a);                
                if( i == p->swipe_start ){
                    k->X = dove::Trajectory(0,0,0); //Initial estimate,  set 0
                    k->P = dove::Trajectory(1,1,1); //set error variance,set 1
                }
                else
                {
                    //time update（prediction）
                    k->X_ = k->X; //X_(k) = X(k-1);
                    k->P_ = k->P+ k->Q; //P_(k) = P(k-1)+Q;
                    // measurement update（correction）
                    k->K = k->P_/ ( k->P_+ k->R ); //gain;K(k) = P_(k)/( P_(k)+R );
                    k->X = k->X_+ k->K * (k->z - k->X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
                    k->P = (dove::Trajectory(1,1,1) - k->K) * k->P_; //P(k) = (1-K(k))*P_(k);
                }
                //smoothed_trajectory.push_back(X);
                k->out_smoothed << i << " " << k->X.x << " " << k->X.y << " " << k->X.a << endl;

                // target - current
                double diff_x = k->X.x - k->x;//
                double diff_y = k->X.y - k->y;
                double diff_a = k->X.a - k->a;

                dx = dx + diff_x;
                dy = dy + diff_y;
                dl.Logger("post from kalman %f %f ", dx, dy);
                smth.at<double>(0,2) = dx;
                smth.at<double>(1,2) = dy;
            } else {
//#define MEMC                
#if defined MEMC
                double pdx = -new_prev_to_cur_transform[vi -1].dx;
                double pdy = -new_prev_to_cur_transform[vi -1].dy;
                double ndx = pdx * 0.5 + dx * 0.1;
                double ndy = pdy * 0.5 + dy * 0.1;
                smth.at<double>(0,2) = ndx;
                smth.at<double>(1,2) = ndy;
                dl.Logger("[%d] will Apply1 %f %f ",i, smth.at<double>(0,2), smth.at<double>(1,2));
                Mat temp;
                cv::warpAffine(refc, temp, smth, refc.size());                
                out << temp;

                smth.at<double>(0,2) = dx;
                smth.at<double>(1,2) = dy;
                dl.Logger("[%d] will Apply2 %f %f ",i, smth.at<double>(0,2), smth.at<double>(1,2));
                cv::warpAffine(refc, refcw, smth, refc.size());
                vi++;
#else
                smth.at<double>(0,2) = dx;
                smth.at<double>(1,2) = dy;
                dl.Logger("[%d] will Apply %f %f ",i, smth.at<double>(0,2), smth.at<double>(1,2));
                ApplyImageRef();
                vi++;
#endif                
            }
        }
        else {
#if defined GPU
            refcg.copyTo(refcwg);
#else
            refc.copyTo(refcw);
#endif
        }

#if defined GPU
        if (compare) {
            cv::resize(refcg, refcg, Size(960, 540));
            cv::resize(refcwg, refcwg, Size(960, 540));
            refcg.copyTo(canvas(Range::all(), Range(0, 960)));
            refcwg.copyTo(canvas(Range::all(), Range(970, 1930)));
#else
        if(compare) {
            cv::resize(refc, refc, Size(960, 540));        
            cv::resize(refcw, refcw, Size(960, 540));
            refc.copyTo(canvas(Range::all(), Range(0, 960)));
            refcw.copyTo(canvas(Range::all(), Range(970, 1930)));
#endif
        }
        else {
#if defined GPU
            refcwg.copyTo(canvas);
#else
            refcw.copyTo(canvas);
#endif
        }

        // sprintf(tx, "%d", i);
        // putText(canvas, tx, Point( 100, 100), FONT_HERSHEY_SIMPLEX, 5, (0), 3);
#if defined GPU
        // out->write(canvas);
        // SetRefCG(src1ocg);
        Mat canvas_t;
        canvas.download(canvas_t);
        out << canvas_t;
        SetRefCG(src1ocg);
#else
        out << canvas;        
        SetRefC(src1oc);
#endif
        i++;
    }
    dl.Logger(".. %f", LapTimer(all));
    return ERR_NONE;

}

void Dove::CalculcateMargin(double minx, double maxx, double miny, double maxy, Rect* mg) {
    int c_lx = 0;
    int c_ly = 0;
    int c_bx = 0;
    int c_by = 0;
    int ww = 0;
    int wh = 0;
    if(abs(int(minx)) > maxx)
        ww = abs(int(minx));
    else 
        ww = maxx;

    if (abs(int(miny)) > maxy)
        wh = abs(int(miny));
    else 
        wh = maxy;

    mg->x = ww;
    mg->y = wh;
    mg->width = p->dst_width - ww*2;
    mg->height = p->dst_height - wh*2;

    dl.Logger("Rect Margin %d %d %d %d", ww, wh, mg->width, mg->height);
}

int Dove::ProcessLK() {
    dl.Logger(" LK process start..");

    VideoCapture in(_in);
    VideoWriter out;    
    out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(p->dst_width, p->dst_height));

#if defined GPU
    cuda::GpuMat src1ocg;
    cuda::GpuMat src1og;
#endif

    Mat src1oc; Mat src1o;
    int i = 0;
    int result = 0;
    int found = 0;

    while(true) {
        in >> src1oc;
        if(src1oc.data == NULL)
            break;
#if defined GPU        
        ImageProcess(src1ocg, src1og);
        src1ocg.download(src1oc);
        src1og.download(src1o);
#else
        ImageProcess(src1oc, src1o);
#endif        
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
            cv::resize(refcw, refcw, Size(p->dst_width, p->dst_height));

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
#if defined GPU
int Dove::ImageProcess(cuda::GpuMat& src, cuda::GpuMat& dst) {
#else
int Dove::ImageProcess(Mat& src, Mat& dst) {
#endif
#if defined GPU
    cuda::GpuMat temp;
    if (p->scale != 1)
        cuda::resize(src, temp, Size(int((float)src.cols / p->scale), int(float(src.rows) / p->scale)), 0, 0, 1);
    else
        src.copyTo(temp);

    if (!p->colored)
        cuda::cvtColor(temp, dst, COLOR_BGR2GRAY);

#else
    Mat temp;
    if(p->scale != 1)
        cv::resize(src, temp, Size(int((float)src.cols/p->scale), int(float(src.rows)/p->scale)), 0,0,1);
    else 
        src.copyTo(temp);

    if(!p->colored)
        cv::cvtColor(temp, temp, COLOR_BGR2GRAY);
    //if tk on? 
    if(!p->run_tracking)
        cv::GaussianBlur(temp, dst, {p->blur_size, p->blur_size}, p->blur_sigma, p->blur_sigma);
    else
        temp.copyTo(dst);

    if (p->has_mask)
        MakeMask();
#endif    

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

    } else if (p->mode == DETECT_TRACKING) {
        float fret = 0.0;
        fret = tck->DetectAndTrack(cur, frame_id, obj, roi);
        dl.Logger("[%d] result %f isFound %d issmae %d ", frame_id, fret, tck->isfound, tck->issame);

        if (tck->isfound == true) {
            if( fret >= p->swipe_threshold * tck->first_summ && swipe_on == false) {
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
            if (tck->issame == true) {
                dl.Logger("[%d] SAME ", frame_id);      
            }

        }    
        else if (tck->isfound == false){ 
            result = TRACK_NONE;
            dl.Logger("[%d] NONE", frame_id);             
        }
    } else if( p->mode == DARKNET_DETECT_MOVE) {
#if defined _MAC_
        Detect(cur, frame_id);
        result = CalculateMove(frame_id);
        if(objects[i].obj_cnt > 0 ) {
             dt.DrawBoxes(refcw, objects[i - 1].bbx);
        }
#endif
    } 
    return result;
}
#if defined _MAC_
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
#endif
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
		k->z = dove::Trajectory(k->x, k->y, k->a);

		if(i == 1){
			k->X = dove::Trajectory(0,0,0); //Initial estimate,  set 0
			k->P = dove::Trajectory(1,1,1); //set error variance,set 1
		}
		else
		{
			//time update（prediction）
			k->X_ = k->X; //X_(k) = X(k-1);
			k->P_ = k->P+ k->Q; //P_(k) = P(k-1)+Q;
			// measurement update（correction）
			k->K = k->P_/ ( k->P_+ k->R ); //gain;K(k) = P_(k)/( P_(k)+R );
			k->X = k->X_+ k->K * (k->z - k->X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
			k->P = (dove::Trajectory(1,1,1) - k->K) * k->P_; //P(k) = (1-K(k))*P_(k);
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
    cv::warpAffine(src, src, smth, src.size());
    // if(scaled == true) {
    //      static int i = 1;
    //      sprintf(filename, "saved/%d_apply.png", i);
    //      imwrite(filename, src);
    //      i++;
    // }
}

void Dove::ApplyImageRef() {
#if defined GPU
    cuda::warpAffine(refcg, refcwg, smth, refcg.size());
#else
    cv::warpAffine(refc, refcw, smth, refc.size());
#endif
}

int Dove::MakeMask() {
    mask = Mat::zeros(p->dst_height, p->dst_width, CV_8UC1);
    mask.setTo(Scalar(255));
    rectangle(mask, Point(p->sx, p->sy), Point(p->sx + p->width, p->sy + p->height), Scalar(0), -1);
    imwrite("saved/mask.png", mask);

    return ERR_NONE;
}

void Dove::ProcessChristmas() {

    VideoCapture in(_in);
    VideoWriter out;
    bool compare = false;
    if (compare)    
        out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(1930, 540));
    else 
        out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(1920, 1080));
    dl.Logger("Process TK started ");
#if defined GPU
    cuda::GpuMat src1ocg;
    cuda::GpuMat src1og;
#endif
    Mat src1oc; Mat src1o;
    int i = 0;
    int result = 0;
    int found = 0;
    vector <TransformParam> prev_to_cur_transform;
    TRACK_OBJ* pre_obj = new TRACK_OBJ();;

    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    
    int t_frame_start = p->swipe_start;
    int t_frame_end = p->swipe_end;

    bool oversampling = false;
    vector<int>same;

    while(true) {
        in >> src1oc;
        if(src1oc.data == NULL)
            break;            

        if(src1oc.cols > p->dst_width)
            cv::resize(src1oc, src1oc, Size(p->dst_width, p->dst_height));

#if defined GPU        
        ImageProcess(src1ocg, src1og);  
        src1ocg.download(src1oc);
        src1og.download(src1o);
#else
        ImageProcess(src1oc, src1o); 
#endif
        if ( i == 0)
        {
            SetRef(src1o);
            SetRefC(src1oc);            
            tck->SetBg(src1o, i);
            i++;
            continue;
        }

        if(i < t_frame_start || i > t_frame_end) {
            SetRef(src1o);
            SetRefC(src1oc);
            out << refc;
            i++;
            continue;            
        }

        if( p->tracker_type != TRACKER_NONE) {
            if (i == t_frame_start)
                tck->TrackerInit(src1o, i, obj, roi);
            else
                tck->TrackerUpdate(src1o, i, obj, roi);            
        }

        //tck->DrawObjectTracking(src1o, obj, roi, false, replay_style);
        
        if (i > t_frame_start && i <= t_frame_end) {
            double dx = 0;
            double dy = 0;
            double da = 0;
        
            vector <Point2f> features1, features2;
            vector <Point2f> goodFeatures1, goodFeatures2;
            vector <uchar> status;
            vector <float> err;
            dl.Logger("tck->rect_feature_roi s %d %d w %d %d ", tck->rect_feature_roi.x, tck->rect_feature_roi.y,
                    tck->rect_feature_roi.width, tck->rect_feature_roi.height);
            Mat roi_ref = ref(tck->rect_feature_roi);
            Mat roi_cur = src1o(tck->rect_feature_roi);
            goodFeaturesToTrack(roi_ref, features1, 200, 0.01, 30);    
            calcOpticalFlowPyrLK(roi_ref, roi_cur, features1, features2, status, err );

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

            dx = affine.at<double>(0,2);
            dy = affine.at<double>(1,2);

            dl.Logger("origin dx %f dy %f", dx ,dy);

            if(p->run_kalman) {
                k->out_transform << i << " " << dx << " " << dy << " " << da << endl;        
                k->x += dx;
                k->y += dy;
                k->a += 0;
                //trajectory.push_back(Trajectory(x,y,a));
                //
                k->out_trajectory << i << " " << k->x << " " << k->y << " " << k->a << endl;
                k->z = dove::Trajectory(k->x, k->y, k->a);

                if(i == 1){
                    k->X = dove::Trajectory(0,0,0); //Initial estimate,  set 0
                    k->P = dove::Trajectory(1,1,1); //set error variance,set 1
                }
                else
                {
                    //time update（prediction）
                    k->X_ = k->X; //X_(k) = X(k-1);
                    k->P_ = k->P+ k->Q; //P_(k) = P(k-1)+Q;
                    // measurement update（correction）
                    k->K = k->P_/ ( k->P_+ k->R ); //gain;K(k) = P_(k)/( P_(k)+R );
                    k->X = k->X_+ k->K * (k->z - k->X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
                    k->P = (dove::Trajectory(1,1,1) - k->K) * k->P_; //P(k) = (1-K(k))*P_(k);
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

            smth.at<double>(0,0) = 1; 
            smth.at<double>(0,1) = 0; 
            smth.at<double>(1,0) = 0; 
            smth.at<double>(1,1) = 1; 
            smth.at<double>(0,2) = dx;
            smth.at<double>(1,2) = dy;
        }

        if (tck->isfound) {
            obj->copy(pre_obj);
        }           

        ApplyImageRef();    
        //printf("refcw apply %f %f \n", smth.at<double>(0,2), smth.at<double>(1,2));
        // sprintf(filename,"%d_warp.png", i);
        // imwrite(filename, refcw);
        out << refcw;

        SetRef(src1o);
        SetRefC(src1oc);
        i++;
    }

    dl.Logger(" All process done.  %f ", LapTimer(all));
}