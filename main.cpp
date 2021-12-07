
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
    
    int mode_dof = 2;
    int mode_cal = 1; //1 : optical flow, 2 : integral + search window
    int mask = 1; //0 : no mask. 1: mask. region
    int result = 0;

    Dove stblz = Dove();

    if(mode_cal == 1) { 
        if(mode_dof == 2) {
            int coord[4] = {0, };
            if(mask == 1) {
                for(int i = 0 ; i < 4 ; i ++)  {
                    coord[i] = atoi(argv[i+2]);
                    printf("coord [%d] %d  \n", i, coord[i]);
                }
            }
            result = stblz.stab_2dof(infile, outfile, coord);            
            
        } else if (mode_dof== 6) {
            result = stblz.stab_6dof(infile, outfile);
        }
    }
    else if(mode_cal == 2) {
        int coord[4];
        for(int i = 0 ; i < 4 ; i ++)  {
            coord[i] = atoi(argv[i+2]);
            printf("coord [%d] %d  \n", i, coord[i]);
        }

        result = stblz.stab_fastwin(infile, outfile, coord);
    }
}
