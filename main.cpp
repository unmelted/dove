
/*****************************************************************************
*                                                                            *
*                            main          								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : main.cpp
    Author(S)       : Me Eunkyung
    Created         : 28 nov 2021

    Description     : main.cpp
    Notes           : video stabil main procedure
*/


#include <stdlib.h>
#include <iostream>
#include "src/DefData.hpp"
#include "src/Stabilization.hpp"

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {

    char infile[40];
    char outfile[40];
    sprintf(infile,"movie/%s.mp4", argv[1]);
    sprintf(outfile, "movie/%s_out2.mp4", argv[1]);

    cout<<infile<<endl;
    cout<<outfile<<endl;

    int coord[4] = {0, };
    if(argc > 2) {
        for(int i = 0 ; i < 4 ; i ++)  {
            coord[i] = atoi(argv[i+2]);
            Logger("coord [%d] %d  \n", i, coord[i]);
        }
    }

    bool has_mask = true;
    int mode = OPTICALFLOW_LK_2DOF;
    //stab_live(infile);
    Dove stblz(mode, has_mask, coord, infile, outfile);
    stblz.Process();
        
    /*
    if( mode == PATH_SMOOTHE) {
        stab_pathsmoothe(infile);
    }
    else if(mode == SIMPLE_KALMAN_LIVE) {
        stab_live(infile);
    }
    else if (mode == OPTICALFLOW_LK_6DOF) {
        stab_2dof(infile, outfile, coord);
    }
    else if (mode == OPTICALFLOW_LK_2DOF) {
        VideoCapture stab(infile);
        VideoWriter output;
        output.open(outfile, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(1920, 1080));
        int result = 0;
        int i = 0;
        if(has_mask == true) {
            for(int i = 0 ; i < 4 ; i ++)  {
                coord[i] = atoi(argv[i+2]);
                Logger("coord [%d] %d  ", i, coord[i]);
            }
        }
    }

        Mat src1oc; Mat src1o;

        while(true) {
            //Logger("loop [%d]", i);

            stab >> src1oc;
            if(src1oc.data == NULL)
                break;
            
            stblz.ImageProcess(src1oc, src1o);

            if ( i == 0)
            {
                stblz.SetRef(src1o);
                i++;
                continue;
            }
        
            result = stblz.CalculateMove(src1o);
            stblz.ApplyImage(src1o);
            stblz.ApplyImage(src1oc, true);

            output << src1oc;        
            // char filename[40];
            // sprintf(filename, "saved/%d_warp.png", i);
            // imwrite(filename, src1oc);

            stblz.SetRef(src1o);        
            i++;
            // if(i == 20)
            //     break;
        }

    }
    else if (mode == TWOPASS) {

    } */
}
