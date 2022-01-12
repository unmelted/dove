  
/*****************************************************************************
*                                                                            *
*                            Util         					    			 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : ExpUtil.cpp
    Author(S)       : Me Eunkyung
    Created         : 10 Jan 2022

    Description     : ExpUtil.Cpp
    Notes           : Uility - json im/export
*/

#include "ExpUtil.hpp"
using json = nlohmann::json;

using namespace dove;
int ExpUtil::ImportVideoInfo(const string js, VIDEO_INFO* info) {
    ifstream json_file(js);
    json j;
    json_file >> j;

    info->input = j["input"];
    info->output = j["output"];
    string ev = j["event"];
    cout << "event string : "<< ev << endl;
    if( ev.compare("FIGURE") == 0 )
        info->event = dove::FIGURE;
    else
        info->event = dove::HOCKEY;
        
    info->width = j["width"];
    info->height = j["height"];
    json sw_period = j["swipePeriod"];
    info->period_cnt = sw_period.size();

    for(auto& el : sw_period) {
        SWIPE_INFO swi;
        swi.order = el["no"];
        swi.start = el["start"];
        swi.end = el["end"];
        swi.target_x = el["target_x"];
        swi.target_y = el["target_y"];
        swi.zoom = el["zoom"];

        info->swipe_period.push_back(swi); 
    }
    std::sort(info->swipe_period.begin(), info->swipe_period.end(), [](SWIPE_INFO a, SWIPE_INFO b) {
              return a.order > b.order;
    });

    cout<< "input : "<<info->input << endl;
    cout<< "output : " <<info->output << endl;
    cout <<"event : " <<info->event << endl;
    return ERR_NONE;
}
/*
void ExpUtil::Export(vector<string>image_paths, vector<SCENE>cal_group, PARAM* p) {

    //world coord 
    //world point1 x = p->world->four_fpt[0].x;
    //world point1 y = p->world->four_fpt[0].x;
    //world center x = p->world->center.x;
    //world center y = p->world->center.y;    

    for (vector<SCENE>::const_iterator it = cal_group.begin(); it != cal_group.end(); it++)
    {
        //id = it-> image_paths[it->id] 
        //point1 x = it->four_fpt[0].x
        //point1 y = it->four_fpt[0].y
        //center x = it->center.x;
        //center y = it->center.y;
    }

    //file write. path = "/saved/xxx.json"

    json jObj = json::object();
    //World
    json world;
    world["X1"] = p->world->four_fpt[0].x;
    world["Y1"] = p->world->four_fpt[0].y;
    world["X2"] = p->world->four_fpt[1].x;
    world["Y2"] = p->world->four_fpt[1].y;
    world["X3"] = p->world->four_fpt[2].x;
    world["Y3"] = p->world->four_fpt[2].y;
    world["X4"] = p->world->four_fpt[3].x;
    world["Y4"] = p->world->four_fpt[3].y;
   
    //jObj["stadium"] = GROUNDTYPE.get();
    jObj["stadium"] = "SoccerHalf";
    jObj["world_coords"] = world;

    //2dPoint
    json point2d = json::object();
    point2d["UpperPosX"] = -1.0;
    point2d["UpperPosY"] = -1.0;
    point2d["MiddlePosX"] = -1.0;
    point2d["MiddlePosX"] = -1.0;
    point2d["LowerPosX"] = -1.0;
    point2d["LowerPosX"] = -1.0;
    //swipe
    json swipe;
    swipe["X1"] = -1.0;
    swipe["Y1"] = -1.0;
    swipe["X2"] = -1.0;
    swipe["Y2"] = -1.0;

    auto arr = json::array();

    for (vector<SCENE>::const_iterator it = cal_group.begin(); it != cal_group.end(); it++)
    {
  
        json jDsc = json::object();
  
        jDsc["dsc_id"] = image_paths[it->id];
        jDsc["pts_2d"] = point2d;

        //3dPoint
        json point3d = json::object();
        point3d["X1"] = it->four_fpt[0].x;
        point3d["Y1"] = it->four_fpt[0].y;
        point3d["X2"] = it->four_fpt[1].x;
        point3d["Y2"] = it->four_fpt[1].y;
        point3d["X3"] = it->four_fpt[2].x;
        point3d["Y3"] = it->four_fpt[2].y;
        point3d["X4"] = it->four_fpt[3].x;
        point3d["Y4"] = it->four_fpt[3].y;
        point3d["CenterX"] = it->center.x;
        point3d["CenterY"] = it->center.y;
   
        jDsc["pts_3d"] = point3d;
        jDsc["pts_swipe"] = swipe;


        arr.push_back(jDsc);

    }
    jObj["points"] = arr;

    //file write
    std::ofstream file("saved/UserPointData_" + getCurrentDateTime("date")  +"_" + getCurrentDateTime("now")+ ".pts");

    file << std::setw(4) << jObj << '\n';
    //std::cout << std::setw(4) << jObj << '\n';
}

void ExpUtil::ExportforApp(vector<string>image_paths, vector<SCENE>cal_group, PARAM* p) {


    json jObj = json::object();
    //World
    json world; json world1;
    world["X1"] = p->world->four_fpt[0].x;
    world["Y1"] = p->world->four_fpt[0].y;
    world["X2"] = p->world->four_fpt[1].x;
    world["Y2"] = p->world->four_fpt[1].y;
    world["X3"] = p->world->four_fpt[2].x;
    world["Y3"] = p->world->four_fpt[2].y;
    world["X4"] = p->world->four_fpt[3].x;
    world["Y4"] = p->world->four_fpt[3].y;

    
    world1["group"] = "Group1";                   //group
    world1["stadium"] = "SoccerHalf";             //stadium
    world1["world_coords"] = world;

    //2dPoint
    json point2d = json::object();
    point2d["UpperPosX"] = -1.0;
    point2d["UpperPosY"] = -1.0;
    point2d["MiddlePosX"] = -1.0;
    point2d["MiddlePosX"] = -1.0;
    point2d["LowerPosX"] = -1.0;
    point2d["LowerPosX"] = -1.0;

    json pt2d = json::object();
    pt2d["IsEmpty"] = false;
    pt2d["X"] = -1.0;
    pt2d["Y"] = -1.0;

    point2d["Upper"] = pt2d;
    point2d["Middle"] = pt2d;
    point2d["Lower"] = pt2d;

    auto world_arr = json::array();
    world_arr.push_back(world1);
    jObj["worlds"] = world_arr;



    auto arr = json::array();

    for (vector<SCENE>::const_iterator it = cal_group.begin(); it != cal_group.end(); it++)
    {

        json jDsc = json::object();

        jDsc["dsc_id"] = image_paths[it->id];
        jDsc["flip"] = 0;
        jDsc["Group"] = "Group1";
        jDsc["Width"] = "w";
        jDsc["Height"] = "h";
        jDsc["infection_point"] = 0;
        jDsc["swipe_base_length"] = -1.0;
        jDsc["ManualOffsetY"] = 0;
        jDsc["FocalLength"] = 0.0;
        jDsc["pts_2d"] = point2d;

        //3dPoint
        json point3d = json::object();
        point3d["X1"] = it->four_fpt[0].x;
        point3d["Y1"] = it->four_fpt[0].y;
        point3d["X2"] = it->four_fpt[1].x;
        point3d["Y2"] = it->four_fpt[1].y;
        point3d["X3"] = it->four_fpt[2].x;
        point3d["Y3"] = it->four_fpt[2].y;
        point3d["X4"] = it->four_fpt[3].x;
        point3d["Y4"] = it->four_fpt[3].y;
        point3d["CenterX"] = it->center.x;
        point3d["CenterY"] = it->center.y;
        json pt3d_1;
        pt3d_1["IsEmpty"] = false;
        pt3d_1["X"] = it->four_fpt[0].x;
        pt3d_1["Y"] = it->four_fpt[0].y;
        point3d["Point1"] = pt3d_1;
        json pt3d_2;
        pt3d_2["IsEmpty"] = false;
        pt3d_2["X"] = it->four_fpt[1].x;
        pt3d_2["Y"] = it->four_fpt[1].y;
        point3d["Point2"] = pt3d_2;
        json pt3d_3;
        pt3d_3["IsEmpty"] = false;
        pt3d_3["X"] = it->four_fpt[2].x;
        pt3d_3["Y"] = it->four_fpt[2].y;
        point3d["Point3"] = pt3d_3;
        json pt3d_4;
        pt3d_4["IsEmpty"] = false;
        pt3d_4["X"] = it->four_fpt[3].x;
        pt3d_4["Y"] = it->four_fpt[3].y;
        point3d["Point4"] = pt3d_4;
        json cent;
        cent["IsEmpty"] = false;
        cent["X"] = it->center.x;
        cent["Y"] = it->center.y;
        point3d["Center"] = cent;

        jDsc["pts_3d"] = point3d;

        arr.push_back(jDsc);

    }
    jObj["points"] = arr;

    //file write
    std::ofstream file("saved/app_UserPointData_" + getCurrentDateTime("date") +"_" + getCurrentDateTime("now") + ".pts");

    file << std::setw(4) << jObj << '\n';
    //cout << std::setw(4) << jObj << '\n';
}
*/

void ExpUtil::TestGetSwipeInfo(string _in, PARAM* p) {
#if defined _MAC_
    if(_in == "movie/4dmaker_600.mp4" || _in == "movie/4dmaker_600_out2.mp4") {
#else
    if (_in == "movie\\4dmaker_600f.mp4" || _in == "movie\\600.mp4") {
#endif
        printf(" ------------ 600 !\n");        
        p->swipe_start = 79; //600 OK        
        p->swipe_end = 180;     
    } else if (_in == "movie/4dmaker_603.mp4") {
        printf(" ------------ 603 !\n");
        p->swipe_start = 79;
        p->swipe_end = 181; //603 -- should conquer - couple gracking -- square merge needed 
    } else if (_in == "movie/4dmaker_626.mp4") {
        printf(" ------------ 626 !\n");        
        p->swipe_start = 79;
        p->swipe_end = 183; //626 OK emerald onepiece single
    } else if (_in == "movie/4dmaker_639.mp4") {
        p->swipe_start = 78;
        p->swipe_end = 130; //639 white shirts man single -- square merge needed
    } else if (_in == "movie/4dmaker_598.mp4") {
        p->swipe_start = 79;
        p->swipe_end = 165; //598 -- frame drop severe
    } else if (_in == "movie/4dmaker_607.mp4") {
        p->swipe_start = 79;
        p->swipe_end = 178; //silver onepiece single - missing during swipe because character
    }else if (_in == "movie/4dmaker_622.mp4") {
        p->swipe_start = 79;
        p->swipe_end = 165; //white onepiece single - missing during swipe because character
    } else if (_in == "movie/4NylanderGoal.mp4") {
        p->swipe_start = 153;
        p->swipe_end = 188; 
        p->roi_sx = 960;
        p->roi_sy = 540;           
    } else if (_in == "movie/BUT_TATAR_4-0.mp4") {
        p->swipe_start = 188;
        p->swipe_end = 232; 
        p->roi_sx = 960;
        p->roi_sy = 540;        
    } else if (_in == "movie/2018_02_09_17_58_50.mp4") {
        p->swipe_start = 64;
        p->swipe_end = 88; //short track
        p->roi_sx = 960;
        p->roi_sy = 540;
    } else if (_in == "movie/2018_02_25_09_55_28.mp4") {
        p->swipe_start = 58;
        p->swipe_end = 91; //short track
        p->roi_sx = 1160; 
        p->roi_sy = 730;
    } else if (_in == "movie/2018_02_13_19_37_53_0.mp4") {
        p->swipe_start = 51;
        p->swipe_end = 84; //short track
        p->roi_sx = 430; 
        p->roi_sy = 850;
    }
}