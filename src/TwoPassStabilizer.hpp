
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
using namespace dove;

class ImageSetForStabilization : public videostab::IFrameSource
{
public:
	std::vector<Mat> imagesForGeneratingTransformationMatrix;	
	std::vector<Mat> imagesForApplyingTransformationMatrix;
	int readCount;
	int readCount_test;	

public:	
	ImageSetForStabilization() {readCount = 0; readCount_test = 0;}
	virtual void reset() {readCount = 0;readCount_test = 0;}

	virtual Mat nextFrame() { return Mat(); }

	void inputImageForGeneratingTransformationMatrix(Mat& image) 
	{ imagesForGeneratingTransformationMatrix.push_back(image.clone()); }
	void inputImageForApplyingTransformationMatrix(Mat& image)
	{ imagesForApplyingTransformationMatrix.push_back(image.clone()); }

	virtual Mat nextFrameForGeneratingTransformationMatrix() 
	{ 			
		Mat frame;		

		imagesForGeneratingTransformationMatrix[readCount].copyTo(frame);				
		if(!frame.empty())
			readCount++;		

		return frame; 		
	}

	virtual Mat nextFrameForApplyingTransformationMatrix() 
	{ 			
		Mat frame;		

		imagesForApplyingTransformationMatrix[readCount_test].copyTo(frame);				
		if(!frame.empty())
			readCount_test++;		

		return frame; 		
	}

	virtual Size size()		
	{return imagesForApplyingTransformationMatrix[readCount_test].size();}

	int width()		{return static_cast<int>(imagesForApplyingTransformationMatrix[readCount].cols);}
	int height()	{return static_cast<int>(imagesForApplyingTransformationMatrix[readCount].rows);}
	int count()		{return static_cast<int>(imagesForApplyingTransformationMatrix.size());}
	double fps()	{return static_cast<int>(30.);}
};

class TwoPass {

public :

    TwoPass();    
    ~TwoPass();

	Ptr<ImageSetForStabilization>					_stabilizedFrames; 
	Ptr<ImageSetForStabilization>		_imageSetForStabilization;		
	Ptr<videostab::MotionEstimatorRansacL2>			_motionEstimation; 		
	Ptr<videostab::KeypointBasedMotionEstimator>    _motionEstBuilder;
	Ptr<videostab::IOutlierRejector>				_outlierRejector;	

	videostab::RansacParams _ransac; 
	videostab::TwoPassStabilizer *tp;

	double _minInlierRatio;
    double _dFrameRate;
    int stStartFrame;
    int stEndFrame;

	int _width;
	int _height;
    double outfps;
    char output[200] = {0, };



   	vector<Mat> frontframe;
	vector<Mat> rearframe;
	vector<Mat> stabframe;

    int ImageBuilder(char* in);
    int StabilizerBuilder();
    int MakeStabilizedVideo(Ptr<ImageSetForStabilization> stabilizedFrames);
    void TranslationAndWarpPerspective(Mat& srcFrame, Mat& dstFrame, Mat& motion, cv::Size size);
};

