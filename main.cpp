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
#include <videostab.h>
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

    Mat src1;
    Mat src2;
    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    

    for(int i = 0; i < images.size() ; i ++) {

        src1 = images[i];
        resize(src1, src1, Size(src1.cols/4, src1.rows/4), 0,0,1);
        cvtColor(src1, src1, COLOR_BGR2GRAY);

        if( i == 0) {
            src2 = src1;
            continue;
        }
        cout<< " i : " <<i << " start " << endl; 

        vector <Point2f> features1, features2;
        vector <Point2f> goodFeatures1, goodFeatures2;
        vector <uchar> status;
        vector <float> err;

        goodFeaturesToTrack(src1, features1, 100, 0.01  , 30 );
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

        cout << "dx : " << dx << "dy : "<< dy << endl;

        double realx = dx;
        double realy = dy;
        double real_sx = ds_x;
        double real_sy = ds_y;

        Point ptsrc;
        Point ptdst;

        ptsrc.x = realx <= 0 ? 0 : int(realx);
        printf(" %d == %d \n", int(realx), ptsrc.x);        
        ptsrc.x = realy <= 0 ? 0 : int(realy);
        int cp_width = src1.cols - int(abs(realx));
        int cp_height = src1.rows - int(abs(realy));

        ptdst.x = realx <= 0 ? int(-realx) : 0;
        ptdst.y = realy <= 0 ? int(-realy) : 0;
        Rect srcrect = Rect(ptsrc.x, ptsrc.y, cp_width, cp_height);
        Rect dstrect = Rect(ptdst.x, ptdst.y, cp_width, cp_height);
        printf("cp_width %d cp_ehgith %d ptsrc.x %d y %d ptdst.x %d y %d \n", cp_width, cp_height, ptsrc.x, ptsrc.y, ptdst.x, ptdst.y);
        Mat tranimg;
        src1(srcrect).copyTo(tranimg(dstrect));

        char filename[30];
        sprintf(filename, "saved/%d_tran.png", i);
        imwrite(filename, tranimg);
        tranimg.release();


        src2.release();
        src2 = src1;
        src1.release();
        
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