  
/*****************************************************************************
*                                                                            *
*                            Util         								     *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Util.hpp
    Author(S)       : Me Eunkyung
    Created         : 10 Jan 2022

    Description     : ExpUtil.hpp
    Notes           : Uility - json im/export
*/

#pragma once 
#include <cmath>
#include "DefData.hpp"
#include "common/json.hpp"

using namespace dove;

class ExpUtil {

public:
    Dlog dl;
    void SetLogFilename(string name) {this->dl.SetLogFilename(name); }; 

    int ImportVideoInfo(const string js, VIDEO_INFO* info);
    void TestGetSwipeInfo(string _in, PARAM* p);

private:

};
