

/*****************************************************************************
*                                                                            *
*                           GodStabil.hpp     								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : GodStabil
    Author(S)       : Me Eunkyung
    Created         : 22 dec 2021

    Description     : 
    Notes           : 
*/

#include "DefData.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/videoio.hpp>


using namespace std;
using namespace cv;

typedef enum 
{
	FHD_SIZE = 0x1,
	UHD_SIZE = 0x2,
	ETC_SIZE = 0x4,
}IMAGE_SIZE;

class ImageSetForStabil : public videostab::IFrameSource
{
public:
	std::vector<Mat> setTransformation;
	std::vector<Mat> setApplying;
	int readCount;
	int readCount_test;	

public:	
	ImageSetForStabil() {readCount = 0; readCount_test = 0;}
	virtual void reset() {readCount = 0;readCount_test = 0;}

	virtual Mat nextFrame() { return Mat(); }

	void InsertForTransformation(Mat& image) 
	{ setTransformation.push_back(image.clone()); }
	void InserForApplying(Mat& image)
	{ setApplying.push_back(image.clone()); }

	virtual Mat nextFrameT() 
	{ 			
		Mat frame;		

		setTransformation[readCount].copyTo(frame);				
		if(!frame.empty())
			readCount++;		

		return frame; 		
	}

	virtual Mat nextFrameA() 
	{ 			
		Mat frame;		

		setApplying[readCount_test].copyTo(frame);				
		if(!frame.empty())
			readCount_test++;		

		return frame; 		
	}

	virtual Size size()		
	{return setTransformation[readCount_test].size();}

	int width()		{return static_cast<int>(setTransformation[readCount].cols);}
	int height()	{return static_cast<int>(setApplying[readCount].rows);}
	int count()		{return static_cast<int>(setTransformation.size());}
	double fps()	{return static_cast<int>(30.);}
};

class GodStabil
{

public:

public:
	GodStabil(IMAGE_SIZE imageSize);
	~GodStabil(void);

public:
	void InitTracking(Mat& image, Rect2d& trackingRegion);
	void UpdateTracking(Mat& image, Rect2d& trackingRegion);

	void RoiImageOfOriginalSize(Mat& srcImage, Mat& dstImage, Rect2d& trackingRegion);
	void RoiImageOfStabilizationSize(Mat& srcImage, Mat& dstImage, Rect2d& trackingRegion);	
		
private:
	bool ModifyRegionToBeValid(Mat& image, Rect2d& inputRegion, Rect2d& paddingRegion, bool cutMargin);
	
public:
	int _samplingScale;
	double _templateScale;

	//Ptr<TrackerKCF> tracker;		
	Ptr<Tracker> tracker;

public:
	void CreateStabilizatedMovie(string outputPath, double outputFps);	

private:
	void IncodingWithStabilizedImage(Ptr<cv::videostab::IFrameSource> stabilizedFrames, string outputPath, double outputFps);

public:
	Ptr<ImageSetForStabil> stabilizedFrames; 
	Ptr<ImageSetForStabil> imageSetForStabilization;		
	Ptr<videostab::MotionEstimatorRansacL2> motionEstimation; 		
	Ptr<videostab::KeypointBasedMotionEstimator> motionEstBuilder;
	Ptr<videostab::IOutlierRejector> outlierRejector;	

	videostab::RansacParams ransac; 
	videostab::TwoPassStabilizer *twoPassStabilizer;
		
	IMAGE_SIZE _imageSize;	
	Size _outputSize;
	double _minInlierRatio;

	int _constraintTrackingDistance;

public:
	double Round(double n);

public:
	void getAdjustMoveCoordination();
	void moveImage(Mat* gMat, int nX, int nY);

	Point2f _baseLinePoint;
	vector<Point2f> _subPoint;
	vector<Mat> testImage;
};