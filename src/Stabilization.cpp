
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
    dl.SetLogFilename("TEST");    
    dl.Logger("instance created.. ");    

#if defined _MAC_
    dt = Detection();
    dt.SetLogger(dl);    
#endif

    _in = infile;
    _out = outfile;

    dl.Logger("Start construct. %s %s  ", _in.c_str(), _out.c_str());
    p->event = event;
    Initialize();
}

Dove::Dove(VIDEO_INFO* vinfo) {
    p = new PARAM();
    t = new TIMER();
    dl = Dlog();
    dl.SetLogFilename("TEST");    
    dl.Logger("instance created.. ");    

#if defined _MAC_
    dt = Detection();
    dt.SetLogger(dl);
#endif
    ConvertToParam(vinfo);
    Initialize();
}

void Dove::ConvertToParam(VIDEO_INFO* info) {

    _in = info->input;
    _out = info->output;
    p->event = info->event;
    si.resize(info->swipe_period.size());
    copy(info->swipe_period.begin(), info->swipe_period.end(), si.begin());


    if(info->period_cnt == 0)
        info->period_cnt = 1;

    if(info->width > 1920)
        p->scale = 2;
    else 
        p->scale = 1;

    if (p->event != FIGURE) {
        p->roi_input = true;
        p->colored = true;
    }
    else {
        p->roi_input = false;
        p->colored = false;        
    }
}

void Dove::Initialize() {
#if defined LOCAL_TEST
    ex.TestGetSwipeInfo(_in, p);
#endif    

    p->mode = DETECT_TRACKING;        
    p->interpolation_mode = SPLINE_LSF; //MEDIAN_KERNEL

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

        p->smoothing_radius = 20;
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

    if(p->interpolation_mode == KALMAN_FILTER) {
        k->Q.set(k->pstd, k->pstd, k->pstd);
        k->R.set(k->cstd, k->cstd, k->cstd);      
    }

    string prev_cur = "analysis/prev_to_cur_transformation.txt";
    string cur_tra = "analysis/trajectory.txt";
    string new_tra = "analysis/smoothed_trajectory.txt";
    string new_delta = "analysis/new_prev_to_cur_transformation.txt";
    out_transform.open(prev_cur);
    out_trajectory.open(cur_tra);
    out_smoothed.open(new_tra);
    out_new.open(new_delta);
    an.out_transform = &out_transform;
    an.out_transform = &out_trajectory;
    an.out_smoothed = &out_smoothed;
    an.out_new = &out_new;

    smth.create(2 , 3 , CV_64F);       
    smth.at<double>(0,0) = 1; 
    smth.at<double>(0,1) = 0; 
    smth.at<double>(1,0) = 0; 
    smth.at<double>(1,1) = 1;      
    dl.Logger("Initialized compelete.");    
}

Dove::~Dove() {
    delete k;
    delete p;
    delete t;
    dl.Logger("delete instance  OK");
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
    cv::Ptr<cudacodec::VideoReader> in = cudacodec::createVideoReader(_in);
#else
    VideoCapture in(_in);
#endif

    dl.Logger("Process TK started ");
#if defined GPU
    cuda::GpuMat src1ocg; 
    cuda::GpuMat src1og;
#endif

    Mat src1oc; Mat src1o;
    int frame_index = 0;
    int swipe_index = 0;
    int result = 0;
    int found = 0;
    vector <TransformParam> prev_to_cur_transform;
    TRACK_OBJ* pre_obj = new TRACK_OBJ();;

    TIMER* tm;
    tm = new TIMER();    
    StartTimer(tm);

    int t_frame_start = si[swipe_index].start;
    int t_frame_end = si[swipe_index].end;

    while(true) {
#if defined GPU
         if (!in->nextFrame(src1ocg))
             break;

#else 
        in >> src1oc;
        if(src1oc.data == NULL)
            break;
#endif

        if ( frame_index == 0)
        {                     
#if defined GPU
            ImageProcess(src1ocg, src1og);
            tck->SetBg(src1og, frame_index);
#else
            ImageProcess(src1oc, src1o);
            tck->SetBg(src1o, frame_index);
#endif
            FRAME_INFO one(frame_index);
            all.push_back(one);
            frame_index++;
            continue;
        }
                
        if(frame_index < t_frame_start || frame_index > t_frame_end) {
            FRAME_INFO one(frame_index);
            all.push_back(one);            
            frame_index++;
            continue;            
        }

#if defined GPU
        ImageProcess(src1ocg, src1og);
#else
        ImageProcess(src1oc, src1o);
#endif

        if (frame_index == t_frame_start) {
#if defined GPU
            if (p->roi_input)
                tck->TrackerInitFx(src1og, frame_index, p->roi_sx, p->roi_sy, obj, roi);
            else
                tck->TrackerInit(src1og, frame_index, obj, roi);
#else
            if(p->roi_input)
                tck->TrackerInitFx(src1o, frame_index, p->roi_sx, p->roi_sy, obj, roi);                
            else 
                tck->TrackerInit(src1o, frame_index, obj, roi);
#endif
            FRAME_INFO one(frame_index, swipe_index, 0, 0);
            all.push_back(one);
        } 
        else {
#if defined GPU            
            tck->TrackerUpdate(src1og, frame_index, obj, roi);
#else
            tck->TrackerUpdate(src1o, frame_index, obj, roi);     
#endif            
        }

        //tck->DrawObjectTracking(src1o, obj, roi, false);
        double dx = 0;
        double dy = 0;
        double da = 0;
        if (frame_index > t_frame_start && frame_index <= t_frame_end) {

            dx = (pre_obj->cx - obj->cx) * p->track_scale;
            dy = (pre_obj->cy - obj->cy) * p->track_scale;
            dl.Logger("pre origin %f %f ", dx, dy);
            FRAME_INFO one(frame_index, swipe_index, dx, dy);
            all.push_back(one);               

            if (tck->isfound)
                obj->copy(pre_obj);
            if(frame_index == t_frame_end) {
#if defined GPU
                tck->SetBg(src1og, frame_index);
#else
                tck->SetBg(src1o, frame_index);
#endif
                swipe_index++;
                t_frame_start = si[swipe_index].start;
                t_frame_end = si[swipe_index].end;
            }
        }
        frame_index++;            
    }

    //dl.Logger("[%d] Image Analysis  %f ", i, LapTimer(all)); 
    Rect mg;     
    MakeNewTrajectory(&mg);

    frame_index = 0;
    swipe_index = 0;
#if defined GPU
    Ptr<cudacodec::VideoReader> in2 = cudacodec::createVideoReader(_in);
    cv::VideoWriter out;
    out.open(_out, VideoWriter::fourcc('A', 'V', 'C', '1'), 30, Size(p->dst_width, p->dst_height));
#else
    VideoCapture in2(_in);
    cv::VideoWriter out;  
    out.open(_out, VideoWriter::fourcc('A', 'V', 'C', '1'), 30, Size(p->dst_width, p->dst_height));
#endif

    while(true) {
#if defined GPU
        if (!in2->nextFrame(src1ocg))
             break;

        //src1ocg.upload(src1oc);
        //ImageProcess(src1ocg, src1og);
        if (src1ocg.cols > p->dst_width)
            cuda::resize(src1ocg, src1ocg, Size(p->dst_width, p->dst_height));

#else 
        in2 >> src1oc;
        if(src1oc.data == NULL)
            break;
        if (src1oc.cols > p->dst_width)
            cv::resize(src1oc, src1oc, Size(p->dst_width, p->dst_height));
#endif

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

#if defined GPU
        cuda::GpuMat canvas = cuda::GpuMat(cv::Size(p->dst_width, p->dst_height), CV_8UC3);
#else
        Mat canvas;
        canvas = Mat::zeros(p->dst_height, p->dst_width, CV_8UC3);
#endif

        if (all[frame_index].onswipe == true) {
            double dx = all[frame_index].new_dx;
            double dy = all[frame_index].new_dy;

            if(p->run_kalman_post) {
                double new_dx = 0;
                double new_dy = 0;
                al.KalmanInOutput(k, &an, dx, dy, frame_index, &new_dx, &new_dy);
                dl.Logger("post from kalman %f %f ", dx, dy);

                smth.at<double>(0,2) = new_dx;
                smth.at<double>(1,2) = new_dy;
            } else {
                smth.at<double>(0,2) = dx;
                smth.at<double>(1,2) = dy;
                dl.Logger("[%d] will Apply %f %f ",i, smth.at<double>(0,2), smth.at<double>(1,2));
            }
            ApplyImageRef();
        }
        else {
#if defined GPU
            refcg.copyTo(refcwg);
#else
            refc.copyTo(refcw);
#endif
        }

#if defined GPU
        refcwg.copyTo(canvas);
        canvas = refcwg(mg);
        cuda::resize(canvas, canvas, Size(p->dst_width, p->dst_height));
#else
        canvas = refcw(mg);
        cv::resize(canvas, canvas, Size(p->dst_width, p->dst_height));
#endif


#if defined GPU
        //out->write(canvas);
        //SetRefCG(src1ocg);
        Mat canvas_t;
        canvas.download(canvas_t);
        //sprintf(filename, "%d_canvas_t.png", i);
        //imwrite(filename, canvas_t);
        out << canvas_t;
        SetRefCG(src1ocg);
#else
        out << canvas;        
        SetRefC(src1oc);
#endif
        frame_index++;
    }

    dl.Logger(".. %f", LapTimer(tm));
    out.release();
    return ERR_NONE;
}

void Dove::CalculcateMargin(double minx, double maxx, double miny, double maxy, Rect* mg) {
    // int mintop = abs(miny);
    // int minleft = abs(minx);
    int mx = max( abs(minx), maxx);
    int my = max( abs(miny), maxy);
    int mintop = my;
    int minleft = mx;
    int minright = p->dst_width - mx;
    int minbottom = p->dst_height - my;
    dl.Logger("top %d left %d right %d bottom %d", mintop, minleft, minright, minbottom);

    if (minleft > mintop * p->dst_width / p->dst_height)
        mintop = minleft * p->dst_height / p->dst_width;
    else
        minleft = mintop * p->dst_width / p->dst_height;
    
    if (minright < minbottom * p->dst_width / p->dst_height)
        minbottom = minright * p->dst_height / p->dst_width;
    else
        minright = minbottom * p->dst_width / p->dst_height;

    mg->x = minleft;
    mg->y = mintop;
    mg->width = minright - minleft;
    mg->height = minbottom - mintop;

    dl.Logger("Rect Margin %d %d %d %d", mg->x, mg->y, mg->width, mg->height);
}

#if defined GPU
int Dove::ImageProcess(cuda::GpuMat& src, cuda::GpuMat& dst) {
#else
int Dove::ImageProcess(Mat& src, Mat& dst) {
#endif
#if defined GPU
    cuda::GpuMat temp;
    if (p->scale != 1) {
        cuda::resize(src, temp, Size(int((float)src.cols / p->scale), int(float(src.rows) / p->scale)), 0, 0, 1);
    }
    else
        src.copyTo(temp);

    if (!p->colored) {
        cuda::GpuMat sub = cuda::GpuMat(Size(temp.cols, temp.rows), CV_8UC1, Scalar(19));
        cuda::cvtColor(temp, temp, cv::COLOR_BGRA2GRAY);
        cuda::subtract(temp, sub, dst);
    }
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

int Dove::MakeNewTrajectory(Rect* mg) {

    dl.Logger("PostPrcess start ... ");
    double a = 0;
    double x = 0;
    double y = 0;
    double minx = p->dst_width;
    double maxx = 0;
    double miny = p->dst_height;
    double maxy = 0;    

    for(int index = 0 ; index < si.size(); index ++) {
        vector<TransformParam>cur_delta;        
        vector<Trajectory>cur_traj;
        vector<Trajectory>smoothed_traj;
        vector<TransformParam>new_delta;
        int findex = 0;

        for(size_t i = si[index].start ; i < si[index].end; i ++) {
            findex = i - si[index].start;
            cur_delta.push_back(TransformParam(all[findex].dx, all[findex].dy, 0));
            x += all[findex].dx;
            y += all[findex].dy;
            a += 0;

            cur_traj.push_back(Trajectory(x, y, a));
            out_transform << i << " " << all[findex].dx << " " << all[findex].dy << " 0" << endl;
            out_trajectory << i << " " << x << " " << y << " " << a << endl;
            findex++;
        }

        if(p->interpolation_mode == SPLINE_LSF) {
            vector<dove::Trajectory> sp_xout;
            vector<dove::Trajectory> sp_yout;    
            al.BSplineTrajectory(cur_traj, &sp_xout, 0);
            al.BSplineTrajectory(cur_traj, &sp_yout, 1);    

            for(size_t i = 0; i < cur_traj.size(); i++) {
                dl.Logger("spline output %f %f ", sp_xout[i].y, sp_yout[i].y);
                smoothed_traj.push_back(dove::Trajectory(sp_xout[i].y, sp_yout[i].y, 0));
                out_smoothed << i << " " << sp_xout[i].y << " " << sp_yout[i].y << " " << "0" << endl;
            }
        }
        else if (p->interpolation_mode == MEDIAN_KERNEL) {
            al.MedianKernel(&an, cur_traj, &smoothed_traj, p->smoothing_radius);
        }

        a = 0;
        x = 0;
        y = 0;

        for(size_t i = si[index].start ; i < si[index].end; i ++) {
            findex = i - si[index].start;            
            x += cur_delta[i].dx;
            y += cur_delta[i].dy;
            a += cur_delta[i].da;

            // target - current
            double diff_x = smoothed_traj[i].x - x;
            double diff_y = smoothed_traj[i].y - y;
            double diff_a = smoothed_traj[i].a - a;

            double dx = cur_delta[i].dx + diff_x;
            double dy = cur_delta[i].dy + diff_y;
            double da = cur_delta[i].da + diff_a;

            new_delta.push_back(TransformParam(dx, dy, da));
            out_new << i << " " << dx << " " << dy << " " << da << endl;
            
            all[findex].new_dx = -1 * dx;
            all[findex].new_dy = -1 * dy;

            if(dx < minx)
                minx = dx;
            if(dx > maxx)
                maxx = dx;
            if(dy < miny)
                miny = dy;
            if (dy > maxy)
                maxy = dy;

            findex++;
        }

        cur_delta.clear();
        cur_traj.clear();
        smoothed_traj.clear();
        new_delta.clear();
    }
    
    dl.Logger("minx %f maxx %f miny %f maxy %f", minx, maxx, miny, maxy);
    CalculcateMargin(minx, maxx, miny, maxy, mg);

    return ERR_NONE;
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
        out_transform << i << " " << dx << " " << dy << " " << da << endl;        
		k->x += dx;
		k->y += dy;
		k->a += da;
		//trajectory.push_back(Trajectory(x,y,a));
		//
		out_trajectory << i << " " << k->x << " " << k->y << " " << k->a << endl;
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
		out_smoothed << i << " " << k->X.x << " " << k->X.y << " " << k->X.a << endl;

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
		out_new << i << " " << dx << " " << dy << " " << da << endl;        
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
        out.open(_out, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(p->dst_width, p->dst_height));
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

        //tck->DrawObjectTracking(src1o, obj, roi, false);
        
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
                out_transform << i << " " << dx << " " << dy << " " << da << endl;        
                k->x += dx;
                k->y += dy;
                k->a += 0;
                //trajectory.push_back(Trajectory(x,y,a));
                //
                out_trajectory << i << " " << k->x << " " << k->y << " " << k->a << endl;
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
                out_smoothed << i << " " << k->X.x << " " << k->X.y << " " << k->X.a << endl;

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
                out_new << i << " " << dx << " " << dy << " " << da << endl;        
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