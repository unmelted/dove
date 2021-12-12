/*****************************************************************************
*                                                                            *
*                           Detection.hpp     								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Detection.hpp
    Author(S)       : Me Eunkyung
    Created         : 11 dec 2021

    Description     : Detection.hpp
    Notes           : Detection class
*/


#include "DefData.hpp"
#include "darknet/yolo_v2_class.hpp"
#include "darknet/darknet.h"

using namespace std;
using namespace cv;


class Detection {

    public:
    Detection(int detector_type = 1);
    ~Detection();

    Dlog dl;
    Detector* dt;
    vector<int>id_filter;
    vector<string> obj_names;

    void SetLogger(Dlog& log) { dl = log; };
    int LoadModel(PARAM* p);
    int Detect(Mat cur, vector<bbox_t>* ret);

    void DrawBoxes(Mat mat_img, vector<bbox_t> result_vec);
    void ShowResult(vector<bbox_t> const result_vec, int frame_id = -1);
    vector<string>ObjectsNamesfromFile(string const filename);
};

