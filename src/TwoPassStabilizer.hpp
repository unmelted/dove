
/*****************************************************************************
*                                                                            *
*                           TwoPassStabilizer.hpp  								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : TwoPassStabilizer.hpp
    Author(S)       : Me Eunkyung
    Created         : 08 dec 2021

    Description     : TwoPassStabilization using opencv videostab
    Notes           : 
*/

#pragma once
#include <opencv2/highgui.hpp>

#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videostab.hpp>
#include "DefData.hpp"

using namespace std;
using namespace cv;

class TwoPass {
    
public :

    TwoPass();    
    ~TwoPasS();

	Ptr<videostab::IFrameSource>					_stabilizedFrames; 
	Ptr<videostab::ImageSetForStabilization>		_imageSetForStabilization;		
	Ptr<videostab::MotionEstimatorRansacL2>			_motionEstimation; 		
	Ptr<videostab::KeypointBasedMotionEstimator>    _motionEstBuilder;
	Ptr<videostab::IOutlierRejector>				_outlierRejector;	

	videostab::RansacParams _ransac; 
	videostab::TwoPassStabilizer *_tp;

	double _minInlierRatio;

	int _width;
	int _height;

}