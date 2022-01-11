
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



#include <iostream>
#include "src/DefData.hpp"
#include "src/Stabilization.hpp"
#include "src/ExpUtil.hpp"

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {

    string jsonfile(argv[1]);
    cout << jsonfile;
    ExpUtil in;
    VIDEO_INFO info;
    int result  = in.ImportVideoInfo(jsonfile, &info);
    if(result == dove::ERR_NONE) {
        Dove stblz(&info);
        result = stblz.Process();
    }

    
    /* old version : cmd input 
    char infile[40];
    char outfile[40];

#if defined _MAC_
    sprintf(infile,"movie/%s.mp4", argv[1]);
    sprintf(outfile, "movie/%s_out2.mp4", argv[1]);
#else
    string inpath;
    cout << "input : ";
    getline(cin, inpath);
    sprintf(infile, "movie\\%s.mp4", inpath.c_str());
    sprintf(outfile, "movie\\%s_out2.mp4", inpath.c_str());
    cout << "main in " << infile << endl;
    cout << "main out " << outfile << endl;
#endif

    cout<<infile<<endl;
    cout<<outfile<<endl;

    int event = dove::FIGURE; 
    Dove stblz(event, has_mask, coord, infile, outfile);
    stblz.Process(); 
    */
}