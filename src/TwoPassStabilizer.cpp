
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
    int cnt = 0;

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

		if(cnt > stEndFrame)
		{
			Mat tempImage = src1oc.clone();
			rearframe.push_back(tempImage);
		}
		else if(cnt < stStartFrame)
		{
			Mat tempImage = src1oc.clone();
			frontframe.push_back(tempImage);
		}
		else
		{			
			blur(roiImage, roiImage, cv::Size(41, 41));

			_imageSetForStabilization->inputImageForGeneratingTransformationMatrix(roiImage);
			_imageSetForStabilization->inputImageForApplyingTransformationMatrix(src1oc);

		}

		cnt++;            
    }

    return true;
}

int TwoPass::StabilizerBuilder() {
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

		//_stabilizedFrames.reset(dynamic_cast<videostab::IFrameSource*>(tp)); // 4-Processing the stabilized frames. The results are showed and saved. 

		MakeStabilizedVideo(_stabilizedFrames); 
	} 
	catch (const exception &e)
	{ 
		Logger("error: %s", e.what()); 
		_stabilizedFrames.release(); 		
	}

    return ERR_NONE;
}

int TwoPass::MakeStabilizedVideo(Ptr<ImageSetForStabilization> stabilizedFrames) {
	int nStartWarpFrame = 0;
	int nMiddleWarpFrame = 0;
	int nEndWarpFrame = 0;

	int nStartEncodingFrame = 0;
	int nMiddleEncodingFrame = 0;
	int nEndEncodingFrame = 0;

	VideoWriter writer; 
	Mat stabilizedFrame; 				
	cv::Size stabilizedSize;

	vector<Mat> stabilizedFrame_; 	

	while (!(stabilizedFrame = stabilizedFrames->nextFrameForGeneratingTransformationMatrix()).empty()) 	
	{
		stabilizedSize = stabilizedFrame.size();

		//ResizeFrameToFHD(stabilizedFrame,stabilizedFrame);
		stabilizedFrame_.push_back(stabilizedFrame);

		nStartWarpFrame++;
	}		

	vector<Mat> testMotion =  tp->stabilizationMotions_;
	for(int i = 0; i < frontframe.size(); i++)
	{			
		TranslationAndWarpPerspective(frontframe[i], frontframe[i], testMotion[0], stabilizedSize);
		//ResizeFrameToFHD(frontframe[i],frontframe[i]);		

		nMiddleWarpFrame++;
	}

	for(int i = 0; i < rearframe.size(); i++)
	{				
		TranslationAndWarpPerspective(rearframe[i], rearframe[i], testMotion[stEndFrame - stStartFrame - 1], stabilizedSize);
		//ResizeFrameToFHD(rearframe[i],rearframe[i]);

		nEndWarpFrame++;
	}

	for(int i = 0; i < frontframe.size(); i++)
	{
		if (!writer.isOpened())
		{
			bool bCheckFileOpen = writer.open(output, VideoWriter::fourcc('m','p','e','g'), outfps, cv::Size(_width, _height));
			if(bCheckFileOpen)
				Logger("Encoding Stabilization File Open Success(Start)");
			else
				Logger("Encoding Stabilization File Open Fail(Start)");
		}
		
		writer << frontframe[i]; 
		nStartEncodingFrame++;
	}

	for(int i = 0; i < stabilizedFrame_.size(); i++)	
	{
		if (!writer.isOpened()) 
		{
			bool bCheckFileOpen = writer.open(output, VideoWriter::fourcc('m','p','e','g'), outfps, cv::Size(_width, _height));
			if(bCheckFileOpen)
				Logger("Encoding Stabilization File Open Success(Middle)");
			else
				Logger("Encoding Stabilization File Open Fail(Middle)");
		}
		
		writer << stabilizedFrame_[i];
		nMiddleEncodingFrame++;
	}

	for(int i = 0; i < rearframe.size(); i++)	
	{
		if (!writer.isOpened()) 	
		{
			bool bCheckFileOpen = writer.open(output, VideoWriter::fourcc('m','p','e','g'), outfps, cv::Size(_width, _height));		
			if(bCheckFileOpen)
				Logger("Encoding Stabilization File Open Success(End)");
			else
				Logger("Encoding Stabilization File Open Fail(End)");
		}

		writer << rearframe[i]; 	
		nEndEncodingFrame++;
	}

    return ERR_NONE;
}


void TwoPass::TranslationAndWarpPerspective(Mat& srcFrame, Mat& dstFrame, Mat& motion, cv::Size size)
{
	int deltaX = (_width - size.width)/2.;
	int deltaY = (_height - size.height)/2.;

	Rect2d rect = Rect2d(
		deltaX,
		deltaY,
		_width - deltaX*2., 
		_height - deltaY*2.);


	warpAffine(srcFrame, dstFrame, motion(cv::Rect(0, 0, 3, 2)), cv::Size(_width,_height), cv::INTER_CUBIC, BORDER_REPLICATE);
}
