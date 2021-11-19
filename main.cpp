#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/flann/flann.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <fstream>
#include <sys/time.h>
#include <ctime>

#include "common/TimeUtil.hpp"


using namespace std;
using namespace cv;


const int HORIZONTAL_BORDER_CROP = 30;

int main() {
    string path = "image/";
    vector<string>image_paths;

    namespace fs = std::__fs::filesystem;

    for (const auto &entry : fs::directory_iterator(path)) {
        if (fs::is_regular_file(entry) &&
           (entry.path().extension().string() == ".png" ||
            entry.path().extension().string() == ".jpg")) {
            image_paths.push_back(entry.path().string());
        }
    }

    sort(begin(image_paths), end(image_paths), less<string>()); 
    vector<Mat> images;
    for (const string &ip : image_paths) {        
        cout << ip << endl;
        images.push_back(imread(ip));
    }

    Mat src1; Mat src1oc; Mat src1o;
    Mat src2;
    Mat smth;
    int cp_width = 0;
    int cp_height = 0;
    Rect srcrect;
    Rect dstrect;
    char filename[30];
    int scale = 4;

    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    
    smth.create(2 , 3 , CV_64F);    

    for(int i = 0; i < images.size() ; i ++) {

        src1oc = images[i];
        int owidth = int(src1oc.cols/4);
        resize(src1oc, src1o, Size(int((float)src1oc.cols/scale), int(float(src1oc.rows)/scale)), 0,0,1);
        cvtColor(src1o, src1o, COLOR_BGR2GRAY);

        if( i == 0) {
            src2 = src1o;
            sprintf(filename, "saved/%d_src1.png", i);
            imwrite(filename, src1oc);
            continue;
        }
        /*
        if(cp_width == 0 ) {
            src1o.copyTo(src1);
        }
        else if(owidth != cp_width) {
            src1 = src1o(dstrect);
        } else {
            src1o.copyTo(src1);            
        }*/

        src1o.copyTo(src1);            
        cout<< " i : " <<i << " start " << endl; 
        // sprintf(filename, "saved/%d_src1.png", i);        
        // imwrite(filename, src1);
        // sprintf(filename, "saved/%d_src2.png", i);        
        // imwrite(filename, src2);

        vector <Point2f> features1, features2;
        vector <Point2f> goodFeatures1, goodFeatures2;
        vector <uchar> status;
        vector <float> err;

        goodFeaturesToTrack(src1, features1, 30, 0.01  , 20 );
        calcOpticalFlowPyrLK(src1, src2, features1, features2, status, err );

        for(size_t i=0; i < status.size(); i++)
        {
            if(status[i])
            {
                goodFeatures1.push_back(features1[i]);
                goodFeatures2.push_back(features2[i]);
            }
        }

//        Mat affine = estimateRigidTransform(goodFeatures1, goodFeatures2, false);
        Mat affine = estimateAffine2D(goodFeatures1, goodFeatures2);

        double dx = affine.at<double>(0,2);
        double dy = affine.at<double>(1,2);
        double da = atan2(affine.at<double>(1,0), affine.at<double>(0,0));
        double ds_x = affine.at<double>(0,0)/cos(da);
        double ds_y = affine.at<double>(1,1)/cos(da);

        cout << "dx : " << dx << " dy : "<< dy << " a : " << da << " dsx : " <<ds_x << " ds_y : " <<ds_y << endl;
        cout << "dx : " << dx * scale<< " dy : "<< dy * scale<< " a : " << da << " dsx : " <<ds_x << " ds_y : " <<ds_y << endl;        

        smth.at<double>(0,0) = ds_x * cos(da);
        smth.at<double>(0,1) = ds_x * -sin(da);
        smth.at<double>(1,0) = ds_y * sin(da);
        smth.at<double>(1,1) = ds_y * cos(da);
        smth.at<double>(0,2) = dx;
        smth.at<double>(1,2) = dy;
        warpAffine(src1, src1, smth, src1.size());        

        smth.at<double>(0,2) = dx * scale;
        smth.at<double>(1,2) = dy * scale;        

        warpAffine(src1oc, src1oc, smth, src1oc.size());
        sprintf(filename, "saved/%d_src1_warp.png", i);
        imwrite(filename, src1oc);
        src1.copyTo(src2);        
/*
        double realx = dx;
        double realy = dy;
        double real_sx = ds_x;
        double real_sy = ds_y;

        Point ptsrc;
        Point ptdst;
        cp_width = src1.cols - int(abs(realx));
        cp_height = src1.rows - int(abs(realy));

        if(realx >= 0) {
            ptsrc.x = realx;
            ptdst.x = 0;
        } else {
            ptsrc.x = 0;
            ptdst.x = -realx;
        }

        if(realy >= 0) {
            ptsrc.y = realy;
            ptdst.y = 0;

        } else {
            ptsrc.y = 0;
            ptdst.y = -realy;
        }

        srcrect = Rect(ptsrc.x, ptsrc.y, cp_width, cp_height);
        dstrect = Rect(ptdst.x, ptdst.y, cp_width, cp_height);
        printf("cp_width %d cp_ehgith %d ptsrc.x %d y %d ptdst.x %d y %d \n", cp_width, cp_height, ptsrc.x, ptsrc.y, ptdst.x, ptdst.y);

        Mat transrc1;
        Mat transrc2;
        //src1(srcrect).copyTo(tranimg(dstrect));
        src2(srcrect).copyTo(transrc2);
        src1(dstrect).copyTo(transrc1);;

        sprintf(filename, "saved/%d_tran_src1.png", i);
        imwrite(filename, transrc1);
        sprintf(filename, "saved/%d_tran_src2.png", i);
        imwrite(filename, transrc2);


        transrc1.copyTo(src2);
*/        
        Logger("[%d] %f ", i, LapTimer(all));
    }

}

/*
int main()
{

    //Create a object of stabilization class
    VideoStab stab;

    //Initialize the VideoCapture object
    VideoCapture cap("1.mp4");
    //VideoCapture cap("videoplayback1.mp4");

    Mat frame_2, frame2;
    Mat frame_1, frame1;



    cap >> frame_1;
    cvtColor(frame_1, frame1, COLOR_BGR2GRAY);

    Mat smoothedMat(2,3,CV_64F);

    VideoWriter outputVideo;
    outputVideo.open("1-1.mp4" , cv::VideoWriter::fourcc('X' , 'V' , 'I' , 'D'), 30 , frame_1.size());



    //int i=0;
    struct timeval time1{};
    struct timeval time2{};

    while(true)
    {

        cap >> frame_2;

        if(frame_2.data == NULL)
        {
            break;
        }

        gettimeofday(&time1, nullptr);
        cvtColor(frame_2, frame2, COLOR_BGR2GRAY);

        Mat smoothedFrame;
        smoothedFrame = stab.stabilize(frame_1 , frame_2);
        outputVideo.write(smoothedFrame);
        //i++;
        //imwrite("file"+ std::to_string(i) +".jpg",smoothedFrame);
        //imwrite("file"+ std::to_string(i) + std::to_string(i) +".jpg",frame_2);
        gettimeofday(&time2, nullptr);
        time_t msec_time = (time2.tv_sec - time1.tv_sec)*1000 + (time2.tv_usec - time1.tv_usec) /1000;
        cout << "msec : " << msec_time << endl;


        //imshow("Stabilized Video" , smoothedFrame);






        char c = (char) waitKey(10);
            if( c == 27 )
              break;


        frame_1 = frame_2.clone();
        frame2.copyTo(frame1);


    }

    return 0;
}
*/