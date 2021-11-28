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
int MakeMask(Mat& mask, int width, int height);

int main(int argc, char* argv[]) {

    char infile[40];
    char outfile[40];
    sprintf(infile,"%s.mp4", argv[1]);
    sprintf(outfile, "%s_out1.mp4", argv[1]);
    cout<<infile<<endl;
    cout<<outfile<<endl;

    VideoCapture stab(infile);
    VideoWriter output;
    output.open(outfile), VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(1920, 1080));    

    Mat src1; Mat src1oc; Mat src1o;
    Mat mask;
    Mat src2;
    Mat smth;

    Mat pre_affine;
    Mat affine;
    int cp_width = 0;
    int cp_height = 0;
    Rect srcrect;
    Rect dstrect;
    char filename[30];
    int scale = 2;
    int i = 0;
    int threshold = 6;

    TIMER* all;
    all = new TIMER();    
    StartTimer(all);    
    smth.create(2 , 3 , CV_64F);    
    MakeMask(mask, 1920/scale, 1080/scale);

    while(true) {

        stab >> src1oc;
        if(src1oc.data == NULL)
            break;

        resize(src1oc, src1o, Size(int((float)src1oc.cols/scale), int(float(src1oc.rows)/scale)), 0,0,1);
        cvtColor(src1o, src1o, COLOR_BGR2GRAY);

        if( i == 0) {
            src2 = src1o;
            sprintf(filename, "saved/%d_src1.png", i);
            imwrite(filename, src1oc);
            i++;
            continue;
        }

        src1o.copyTo(src1);            
        //cout<< " i : " <<i << " start " << endl; 
        // sprintf(filename, "saved/%d_src1.png", i);        
        // imwrite(filename, src1);
        // sprintf(filename, "saved/%d_src2.png", i);        
        // imwrite(filename, src2);

        vector <Point2f> features1, features2;
        vector <Point2f> goodFeatures1, goodFeatures2;
        vector <uchar> status;
        vector <float> err;

        goodFeaturesToTrack(src1, features1, 30, 0.01  , 30, noArray(), 11, false, 0.04);
        calcOpticalFlowPyrLK(src1, src2, features1, features2, status, err );

        for(size_t i=0; i < status.size(); i++)
        {
            if(status[i])
            {
                goodFeatures1.push_back(features1[i]);
                goodFeatures2.push_back(features2[i]);
            }
        }

        if(goodFeatures1.size() < threshold || goodFeatures2.size() < threshold) {
             cout<< i << " no feature to track.. feature cnt : "<< goodFeatures1.size() << endl;
             sprintf(filename, "%d_no_feature.png", i);
             imwrite(filename, src1oc);
             pre_affine.copyTo(affine);
         }
         else {
            //affine = estimateAffine2D(goodFeatures1, goodFeatures2);
            affine = estimateRigidTransform(goodFeatures1, goodFeatures2, false);                
        }

        double dx = affine.at<double>(0,2);
        double dy = affine.at<double>(1,2);
        double da = atan2(affine.at<double>(1,0), affine.at<double>(0,0));
        double ds_x = affine.at<double>(0,0)/cos(da);
        double ds_y = affine.at<double>(1,1)/cos(da);

        smth.at<double>(0,0) = 1; //ds_x * cos(da);
        smth.at<double>(0,1) = 0; //ds_x * -sin(da);
        smth.at<double>(1,0) = 0; //ds_y * sin(da);
        smth.at<double>(1,1) = 1; //ds_y * cos(da);
        smth.at<double>(0,2) = dx;
        smth.at<double>(1,2) = dy;

        warpAffine(src1, src1, smth, src1.size());        
        smth.at<double>(0,2) = dx * scale;
        smth.at<double>(1,2) = dy * scale;      
        warpAffine(src1oc, src1oc, smth, src1oc.size());
/*
        double realx = dx * scale;
        double realy = dy * scale;
        double real_sx = ds_x;
        double real_sy = ds_y;

        Point ptsrc;
        Point ptdst;
        cp_width = src1oc.cols - int(abs(realx));
        cp_height = src1oc.rows - int(abs(realy));

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
        src1oc(dstrect).copyTo(transrc1);
        src1(dstrect).copyTo(transrc2);

        char filename[30];
        sprintf(filename, "saved/%d_tran_src1.png", i);
        imwrite(filename, transrc1);
        sprintf(filename, "saved/%d_tran_src2.png", i);
        imwrite(filename, transrc2);
*/
        output << src1oc;
        src1.copyTo(src2);     
        affine.copyTo(pre_affine);   

        //transrc1.copyTo(src2);
        
        Logger("[%d] %f ", i, LapTimer(all));
    }

}

int MakeMask(Mat& mask, int width, int height) {
    cout<< "mask width , height : " << width << " , "<< height<<endl;
    int border = width/12;
    mask = Mat::zeros(height, width, CV_8UC1);
    rectangle(mask, Point(631, 247), Size(300, 200), Scalar(255), -1);
    imwrite("mask.png", mask);
}
