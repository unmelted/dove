
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
	_imageSetForStabilization = makePtr<ImageSetForStabilization>();
	_motionEstimation = makePtr<videostab::MotionEstimatorRansacL2>(videostab::MM_TRANSLATION);
	_ransac = _motionEstimation->ransacParams(); 
	tp = new videostab::TwoPassStabilizer(); 

	_motionEstBuilder = makePtr<videostab::KeypointBasedMotionEstimator>(_motionEstimation); 
	_outlierRejector = makePtr<videostab::NullOutlierRejector>(); 

	_minInlierRatio = 0.1; 

}

TwoPass::~TwoPass() {

}

int TwoPass::ImageBuilder(char* in) {
    VideoCapture stab(in);
    while(true) {
        Mat src1oc;
        stab >> src1oc;
		if(src1oc.empty()) break;

		_width = src1oc.cols;
		_height = src1oc.rows;

		Mat roiImage;
		Rect2d rect = Rect2d(
			src1oc.cols/4*1, 
			src1oc.rows/3*1, 
			src1oc.cols/4*2, 
			src1oc.rows/3*1);

		src1oc(rect).copyTo(roiImage);
            
    }
}

int TwoPass::StabilizerBuilder(char* out) {
	//_imageSetForStabilization->inputImageForGeneratingTransformationMatrix(Mat());
	//_imageSetForStabilization->inputImageForApplyingTransformationMatrix(Mat());

	try {
		_ransac.size = 3; 
		_ransac.thresh = 5; 
		_ransac.eps = 0.5; 

		_motionEstimation->setRansacParams(_ransac); 
		_motionEstimation->setMinInlierRatio(_minInlierRatio); // second, create a feature detector 		

		_motionEstBuilder->setOutlierRejector(_outlierRejector); // 3-Prepare the stabilizer 

		int radius_pass = 15;		
		bool est_trim = true; 

		tp->setEstimateTrimRatio(est_trim); 
		tp->setMotionStabilizer(makePtr<videostab::GaussianMotionFilter>(radius_pass)); 		

		int radius = 15; 
		double trim_ratio = 0.1; 
		bool incl_constr = false; 						

		tp->setFrameSource(_imageSetForStabilization); 		
		tp->setMotionEstimator(_motionEstBuilder); 
		tp->setRadius(radius); 
		tp->setTrimRatio(trim_ratio); 
		tp->setCorrectionForInclusion(incl_constr); 
		tp->setBorderMode(BORDER_REPLICATE);		

		_stabilizedFrames.reset(dynamic_cast<videostab::IFrameSource*>(tp)); // 4-Processing the stabilized frames. The results are showed and saved. 

		MakeStabilizedImage(_stabilizedFrames); 
	} 
	catch (const exception &e) 
	{ 
		Logger("error: %s", e.what()); 
		_stabilizedFrames.release(); 		
	}

}

int TwoPass::MakeStabilizedImage(Ptr<videostab::IFrameSource> stabilizedFrames) {

}