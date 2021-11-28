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
    int mode_dof = 2;
    int result = 0;

    if(mode_dof == 2) {
        //result = stab_2dof(infile, outfile);
        cout <<"TBD" << endl;
    } else if (mode_dof== 6) {
        cout<<"..."<<endl;
    }

}
