
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


Detection::Detection(int detector_type) {
}

Detection::~Detection() {

}

int Detection::LoadModel(PARAM* p)
{
    if (p->detector_type == DARKNET_YOLOV4) {
        Detector yolo(p->cfg_file, p->weights_file);
        auto obj_names = ObjectsNamesfromFile(p->names_file);
        string out_videofile = "result.avi";
        bool const save_output_videofile = false;   // true - for history
        bool const send_network = false;        // true - for remote detection
        bool const use_kalman_filter = false;   // true - for stationary camera
        detector = &yolo;
    }
    return ERR_NONE;
}

void Detection::DrawBoxes(Mat mat_img, vector<bbox_t> result_vec, vector<string> obj_names,
    int current_det_fps, int current_cap_fps)
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
            //max_width = max(max_width, 283);
            string coords_3d;
            if (!isnan(i.z_3d)) {
                stringstream ss;
                ss << fixed << setprecision(2) << "x:" << i.x_3d << "m y:" << i.y_3d << "m z:" << i.z_3d << "m ";
                coords_3d = ss.str();
                Size const text_size_3d = getTextSize(ss.str(), FONT_HERSHEY_COMPLEX_SMALL, 0.8, 1, 0);
                int const max_width_3d = (text_size_3d.width > i.w + 2) ? text_size_3d.width : (i.w + 2);
                if (max_width_3d > max_width) max_width = max_width_3d;
            }

            rectangle(mat_img, Point2f(max((int)i.x - 1, 0), max((int)i.y - 35, 0)),
                Point2f(min((int)i.x + max_width, mat_img.cols - 1), min((int)i.y, mat_img.rows - 1)),
                color, FILLED, 8, 0);
            putText(mat_img, obj_name, Point2f(i.x, i.y - 16), FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(0, 0, 0), 2);
            if(!coords_3d.empty()) putText(mat_img, coords_3d, Point2f(i.x, i.y-1), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 0), 1);
        }
    }
    if (current_det_fps >= 0 && current_cap_fps >= 0) {
        string fps_str = "FPS detection: " + to_string(current_det_fps) + "   FPS capture: " + to_string(current_cap_fps);
        putText(mat_img, fps_str, Point2f(10, 20), FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(50, 255, 0), 2);
    }
}

void Detection::ShowConsoleResult(vector<bbox_t> const result_vec, vector<string> const obj_names, int frame_id) {
    if (frame_id >= 0) cout << " Frame: " << frame_id << endl;
    for (auto &i : result_vec) {
        if (obj_names.size() > i.obj_id) cout << obj_names[i.obj_id] << " - ";
        cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
            << ", w = " << i.w << ", h = " << i.h
            << setprecision(3) << ", prob = " << i.prob << endl;
    }
}

vector<string> Detection::ObjectsNamesfromFile(string const filename) {
    ifstream file(filename);
    vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(string line; getline(file, line);) file_lines.push_back(line);
    cout << "object names loaded \n";
    return file_lines;
}
