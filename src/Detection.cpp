
/*****************************************************************************
*                                                                            *
*                           Detection.cpp     								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Detection.cpp
    Author(S)       : Me Eunkyung
    Created         : 11 dec 2021

    Description     : Detection.cpp
    Notes           : Detection class
*/

#include "Detection.hpp"


Detection::Detection() {

}

Detection::~Detection() {

}

int Detection::LoadModel(PARAM* _p)
{
    p = _p;
    if (p->detector_type == DARKNET_YOLOV4) {
        dt = new Detector(p->cfg_file, p->weights_file);
        obj_names = ObjectsNamesfromFile(p->names_file);
        bool const save_output_videofile = false;   // true - for history
        bool const send_network = false;        // true - for remote detection
        bool const use_kalman_filter = false;   // true - for stationary camer
    }

    if(p->id_filter.size() > 0) {
        id_filter.resize(p->id_filter.size());
        copy(p->id_filter.begin(), p->id_filter.end(), id_filter.begin());
    }

   return ERR_NONE;
}

int Detection::Detect(Mat cur, vector<bbox_t>* ret) {

    vector<bbox_t>box;  
    box = dt->detect(cur);
    //box = dt->tracking_id(box);
    dl.Logger("Detection::Detect done %d ", box.size());
    if(box.size() > 0 ){
        for(int i = 0 ; i < box.size(); i ++) {
            //dl.Logger("detected id %d size %d %d ", box[i].obj_id, box[i].w, box[i].h);
            if(box[i].obj_id == 0 && box[i].prob > p->detect_threshold) {
                if (p->has_mask == false)
                    ret->push_back(box[i]);
                else if(p->has_mask == true && IsObjInROI(box[i]) == true)
                    ret->push_back(box[i]);
            }
        }   
    }
    else ret->clear();
    dl.Logger("Detection::Detect filtered %d ", ret->size());
    return ERR_NONE;
}

bool Detection::IsObjInROI(bbox_t& b) {
//    dl.Logger("in %d %d %d %d mask %d %d %d %d ", b.x, b.y, b.w, b.h,
//        p->sx, p->sy, p->width, p->height);
    if(b.x /p->scale< p->sx || b.y /p->scale< p->sy )
        return false;
    if(b.x /p->scale+ b.w/p->scale > (p->sx + p->width) ||
        b.y/p->scale + b.h/p->scale > (p->sy + p->height))
        return false;

    return true;
}

void Detection::DrawBoxes(Mat mat_img, vector<bbox_t> result_vec)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        Scalar color = obj_id_to_color(i.obj_id);
        rectangle(mat_img, Rect(i.x, i.y, i.w, i.h), color, 2);
        if (obj_names.size() > i.obj_id) {
            string obj_name = obj_names[i.obj_id];
            if (i.track_id > 0) obj_name += " - " + to_string(i.track_id);
            Size const text_size = getTextSize(obj_name, FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
            max_width = max(max_width, (int)i.w + 2);
            
            rectangle(mat_img, Point2f(max((int)i.x - 1, 0), max((int)i.y - 35, 0)),
                Point2f(min((int)i.x + max_width, mat_img.cols - 1), min((int)i.y, mat_img.rows - 1)),
                color, FILLED, 8, 0);
            putText(mat_img, obj_name, Point2f(i.x, i.y - 16), FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(0, 0, 0), 2);
        }
    }
}

void Detection::ShowResult(vector<bbox_t> const result_vec, int frame_id) {
    for (auto &i : result_vec) {
        if (obj_names.size() > i.obj_id) cout << obj_names[i.obj_id] << " - ";
        cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
            << ", w = " << i.w << ", h = " << i.h
            << setprecision(3) << ", prob = " << i.prob << endl;
    }
}

vector<string> Detection::ObjectsNamesfromFile(string const filename) {
    ifstream file(filename);
    vector<string> file_lines;
    if (!file.is_open()) return file_lines;
    for(string line; getline(file, line);) file_lines.push_back(line);
    return file_lines;
}
