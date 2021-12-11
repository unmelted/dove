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

    void* detector;

    int LoadModel(PARAM* p);
    void DrawBoxes(Mat mat_img, vector<bbox_t> result_vec, vector<string> obj_names,
        int current_det_fps = -1, int current_cap_fps = -1);
    void ShowConsoleResult(vector<bbox_t> const result_vec, vector<string> const obj_names, int frame_id = -1);
    vector<string>ObjectsNamesfromFile(string const filename);
};

