
/*****************************************************************************
*                                                                            *
*                           TwoPassStabilizer.cpp  								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : TwoPassStabilizer.cpp
    Author(S)       : Me Eunkyung
    Created         : 08 dec 2021

    Description     : TwoPassStabilization using opencv videostab
    Notes           : 
*/

#include "TwoPassStabilizer.hpp"


TwoPass::TwoPass() {
	_imageSetForStabilization = makePtr<videostab::ImageSetForStabilization>();
	_motionEstimation = makePtr<videostab::MotionEstimatorRansacL2>(videostab::MM_TRANSLATION);
	_ransac = _motionEstimation->ransacParams(); 
	_twoPassStabilizer = new videostab::TwoPassStabilizer(); 

	_motionEstBuilder = makePtr<videostab::KeypointBasedMotionEstimatorGpu>(_motionEstimation); 
	_outlierRejector = makePtr<videostab::NullOutlierRejector>(); 

	_minInlierRatio = 0.1; 

}

TwoPass::TwoPasS() {

}