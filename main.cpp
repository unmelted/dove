
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


#include "stab.hpp"

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {

    char infile[40];
    char outfile[40];
    sprintf(infile,"%s.mp4", argv[1]);
    sprintf(outfile, "%s_out1.mp4", argv[1]);

    cout<<infile<<endl;
    cout<<outfile<<endl;

    int mode_dof = 2;
    int mode_cal = 2; //1 : optical flow, 2 : integral + search window
    int result = 0;

    if(mode_cal == 1) { 
        if(mode_dof == 2) {
            result = stab_2dof(infile, outfile);        
        } else if (mode_dof== 6) {
            result = stab_6dof(infile, outfile);
        }
    }
    else if(mode_cal == 2) {
        int coord[4];
        for(int i = 0 ; i < 4 ; i ++)  {
            char t[2];
            strcpy(argv[i+2], t);
            coord[i] = t[0] - 'a';
        }

        cout<< coord <<endl;
        result = stab_fastwin(infile, outfile, coord);
    }
}
