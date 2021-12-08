
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
    sprintf(outfile, "movie/%s_out1.mp4", argv[1]);

    cout<<infile<<endl;
    cout<<outfile<<endl;
    //VideoCapture stab(infile);
    //VideoWriter output;
    //output.open(outfile, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(1920, 1080));    

    int coord[4] = {0, };
    for(int i = 0 ; i < 4 ; i ++)  {
        coord[i] = atoi(argv[i+2]);
        printf("coord [%d] %d  \n", i, coord[i]);
    }

    stab_2dof(infile, outfile, coord);

// Second step
/*
    int mode = OPTICALFLOW_LK_2DOF;
    bool has_mask =true;
    int result = 0;
    int i = 0;
    if(has_mask == true) {
        for(int i = 0 ; i < 4 ; i ++)  {
            coord[i] = atoi(argv[i+2]);
            printf("coord [%d] %d  \n", i, coord[i]);
        }
    }

    Dove stblz = Dove(mode, has_mask, coord);
    Mat src1oc; Mat src1o;

    while(true) {
        printf("loop [%d] \n", i);
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
        if(result > 0 ) {
            stblz.ApplyImage(src1o);
            stblz.ApplyImage(src1oc, true);
        }
        stblz.SetRef(src1o);        
        output << src1oc;    
        i++;
    }
*/
}
