
/*****************************************************************************
*                                                                            *
*                           GodStabil.cpp     								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : GodStabil
    Author(S)       : Me Eunkyung
    Created         : 22 dec 2021

    Description     : 
    Notes           : 
*/

#include "GodStabil.hpp"

GodStabil::GodStabil(IMAGE_SIZE imageSize)
{
	//Tracking init
	//param.desc_pca = TrackerKCF::GRAY | TrackerKCF::CN;
	//param.desc_npca = 0;
	//param.compress_feature = true;
	//param.compressed_size = 2;	
	//
	//tracker = TrackerKCF::createTracker(param);	
	//tracker->setFeatureExtractor(SobelExtractor);

	tracker =  TrackerCSRT::create();

	//Stabilization init
	imageSetForStabilization = makePtr<ImageSetForStabil>();
	motionEstimation = makePtr<cv::videostab::MotionEstimatorRansacL2>(cv::videostab::MM_TRANSLATION); 		
	ransac = motionEstimation->ransacParams(); 
	twoPassStabilizer = new cv::videostab::TwoPassStabilizer(); 

	motionEstBuilder = makePtr<cv::videostab::KeypointBasedMotionEstimator>(motionEstimation); 
	outlierRejector = makePtr<cv::videostab::NullOutlierRejector>(); 

	_minInlierRatio = 0.1; 

	_imageSize = imageSize;

	if(_imageSize == FHD_SIZE)
	{
		_outputSize = Size(1920, 1080);
		_samplingScale = 2.0;
		_templateScale = 2.0;
		_constraintTrackingDistance = 4000/_samplingScale;
	}
	else if(_imageSize == UHD_SIZE)
	{
		_outputSize = Size(3840, 2160);
		_samplingScale = 4.0;
		_templateScale = 4.0;
		_constraintTrackingDistance = 8000/_samplingScale;
	}
	else
	{			
		_samplingScale = 4.0;
		_templateScale = 2.0;
		_constraintTrackingDistance = 8000/_samplingScale;
	}
}

GodStabil::~GodStabil(void)
{
	if (!tracker.empty())
		tracker.release();

	if (!stabilizedFrames.empty())
		stabilizedFrames.release();

	if (!imageSetForStabilization.empty())
		imageSetForStabilization.release();

	if (!motionEstimation.empty())
		motionEstimation.release();

	if (!motionEstBuilder.empty())
		motionEstBuilder.release();

	if (!outlierRejector.empty())
		outlierRejector.release();

	return;
}


void GodStabil::InitTracking(Mat& image, Rect2d& trackingRegion)
{		
	tracker->init( image, trackingRegion);

	//ModifyRegionToBeValid(image, trackingRegion, Rect(), false);

	_baseLinePoint.x = trackingRegion.x;
	_baseLinePoint.y = trackingRegion.y;
}

void GodStabil::UpdateTracking(Mat& image, Rect2d& trackingRegion)
{
	float prevDistance = (sqrt(pow((float)trackingRegion.x, 2) + pow((float)trackingRegion.y, 2)));
	Rect temp;// = trackingRegion;

	tracker->update( image, temp);

	Point2f _subPointTest;
	if(temp.width != 0)
		//_subPointTest = Point2f((trackingRegion.x-temp.x), (trackingRegion.y-temp.y));
		_subPointTest = Point2f((trackingRegion.x-_baseLinePoint.x), (trackingRegion.y-_baseLinePoint.y));
	else
		_subPointTest = Point2f();
	_subPoint.push_back(_subPointTest);

    Rect2d t;
	if(ModifyRegionToBeValid(image, trackingRegion, t, false))
	{
		tracker = TrackerKCF::create();	
		//tracker->setFeatureExtractor(SobelExtractor);
		tracker->init( image, trackingRegion);
	}

	float currDistance = (sqrt(pow((float)trackingRegion.x, 2) + pow((float)trackingRegion.y, 2)));	
	float distance = abs(prevDistance - currDistance);

	if(distance > _constraintTrackingDistance)
	{
		trackingRegion = temp;
		tracker = TrackerCSRT::create();
		tracker->init( image, trackingRegion);
	}
}

bool GodStabil::ModifyRegionToBeValid(Mat& image, Rect2d& inputRegion, Rect2d& paddingRegion, bool cutMargin)
{
	bool isModifiedRegion = false;
	paddingRegion = inputRegion; 
	paddingRegion.x = 0; 
	paddingRegion.y = 0;

	if( cutMargin == false) 
	{
		if(inputRegion.x + inputRegion.width > image.cols)
		{
			inputRegion.x = image.cols - inputRegion.width;		
			isModifiedRegion = true;
		}
		if(inputRegion.y + inputRegion.height > image.rows)
		{
			inputRegion.y = image.rows - inputRegion.height;		
			isModifiedRegion = true;
		}
		if(inputRegion.x < 0)
		{
			inputRegion.x = 0;
			isModifiedRegion = true;
		}
		if(inputRegion.y < 0)
		{
			inputRegion.y = 0;	
			isModifiedRegion = true;
		}
	}
	else // Def
	{
		if(inputRegion.x + inputRegion.width > image.cols)
		{
			inputRegion.width -= inputRegion.x + inputRegion.width - image.cols;
			paddingRegion.width = inputRegion.width;
			isModifiedRegion = true;
		}
		if(inputRegion.y + inputRegion.height > image.rows)
		{
			inputRegion.height -= inputRegion.y + inputRegion.height- image.rows;
			paddingRegion.height = inputRegion.height;
			isModifiedRegion = true;
		}
		if(inputRegion.x < 0)
		{
			paddingRegion.width += inputRegion.x;
			inputRegion.width = paddingRegion.width;
			paddingRegion.x = abs(_outputSize.width/_templateScale - paddingRegion.width);
			inputRegion.x = 0;			
			isModifiedRegion = true;
		}
		if(inputRegion.y < 0)
		{
			paddingRegion.height += inputRegion.y;
			inputRegion.height = paddingRegion.height;
			paddingRegion.y = abs(_outputSize.height/_templateScale - paddingRegion.height);
			inputRegion.y = 0;
			isModifiedRegion = true;
		}		
	}	
	return isModifiedRegion;
}


void GodStabil::RoiImageOfOriginalSize(Mat& srcImage, Mat& dstImage, Rect2d& trackingRegion)
{
	srcImage(Rect2d(
		trackingRegion.x * _samplingScale, 
		trackingRegion.y * _samplingScale, 
		trackingRegion.width * _samplingScale, 
		trackingRegion.height * _samplingScale)).copyTo(dstImage);
}

void GodStabil::RoiImageOfStabilizationSize(Mat& srcImage, Mat& dstImage, Rect2d& stabilizationRegion)
{	
	//트래킹 영역의 중심을 기준으로
	Point2d centerPoint = 
		Point2d((stabilizationRegion.x*_samplingScale + (stabilizationRegion.width*_samplingScale/2)), 
		(stabilizationRegion.y*_samplingScale + (stabilizationRegion.height*_samplingScale/2)));
	stabilizationRegion = Rect2d( 
		centerPoint.x - _outputSize.width / (_templateScale*2),
		centerPoint.y - _outputSize.height / (_templateScale*2),
		_outputSize.width / _templateScale,
		_outputSize.height / _templateScale);

	// The revision to remove the padding images
	/*
	oooooooxxxx                 ooooooooooo
	oooooooxxxx	                ooooooooooo
	oooooooxxxx      ---->      ooooooooooo
	oooooooxxxx                 ooooooooooo
	*/

	if (stabilizationRegion.x < 0)
		stabilizationRegion.x = 0;
	if (stabilizationRegion.x + stabilizationRegion.width > srcImage.cols)
		stabilizationRegion.x = srcImage.cols - stabilizationRegion.width;
	if (stabilizationRegion.y < 0)
		stabilizationRegion.y = 0;
	if (stabilizationRegion.y + stabilizationRegion.height > srcImage.rows)
		stabilizationRegion.y = srcImage.rows - stabilizationRegion.height;

	/*
	//원 영상의 중심점을 기준으로 스테빌라이져					
	//if(count == 0)
	//{
	//	count ++;
	//	return;
	//}
	//Point2d centerPoint = 
	//	Point2d(((-subX[count])*_samplingScale*2 + (stabilizationRegion.width*_samplingScale/2)), 
	//			((-subY[count])*_samplingScale*2 + (stabilizationRegion.height*_samplingScale/2)));
	//count++;	
	//
	//
	//stabilizationRegion = Rect2d( 
	//	centerPoint.x - _outputSize.width / (_templateScale*2),
	//	centerPoint.y - _outputSize.height / (_templateScale*2),
	//	_outputSize.width / _templateScale,
	//	_outputSize.height / _templateScale);
	*/
	
	srcImage(stabilizationRegion).copyTo(dstImage);

	/*
	Rect2d padding;
	bool isModifiedRegion = ModifyRegionToBeValid(srcImage, stabilizationRegion, padding, true);

	//Rect stabilizationRegion_ = stabilizationRegion;
	//Rect padding_ = padding;

	if(isModifiedRegion == true)		
	{	
		Mat temp(Size(Round(_outputSize.width/_templateScale), 
			Round(_outputSize.height/_templateScale)), CV_8UC3, cv::Scalar(0));
		srcImage(stabilizationRegion).copyTo(temp(padding));
		temp.copyTo(dstImage);		
	}	
	else
		srcImage(stabilizationRegion).copyTo(dstImage);		
	*/

	return;
}

void GodStabil::CreateStabilizatedMovie(string outputPath, double outputFps)
{
	// imageSetForStabilization->inputImageForGeneratingTransformationMatrix(Mat());
	// imageSetForStabilization->inputImageForApplyingTransformationMatrix(Mat());

	try {
		ransac.size = 3; 
		ransac.thresh = 5; 
		ransac.eps = 0.5; 

		motionEstimation->setRansacParams(ransac); 
		motionEstimation->setMinInlierRatio(_minInlierRatio); // second, create a feature detector 		

		motionEstBuilder->setOutlierRejector(outlierRejector); // 3-Prepare the stabilizer 

		int radius_pass = 15;		
		bool est_trim = true; 

		twoPassStabilizer->setEstimateTrimRatio(est_trim); 
		twoPassStabilizer->setMotionStabilizer(makePtr<cv::videostab::GaussianMotionFilter>(radius_pass)); 		

		int radius = 15; 
		double trim_ratio = 0.1; 
		bool incl_constr = false; 						

		twoPassStabilizer->setFrameSource(imageSetForStabilization); 		
		twoPassStabilizer->setMotionEstimator(motionEstBuilder); 
		twoPassStabilizer->setRadius(radius); 
		twoPassStabilizer->setTrimRatio(trim_ratio); 
		twoPassStabilizer->setCorrectionForInclusion(incl_constr); 
		twoPassStabilizer->setBorderMode(BORDER_REPLICATE);		

		stabilizedFrames.reset(make_shared<ImageSetForStabil*>(twoPassStabilizer)); // 4-Processing the stabilized frames. The results are showed and saved. 
		IncodingWithStabilizedImage(stabilizedFrames, outputPath, outputFps); 	
	} 
	catch (const exception &e) 
	{ 
		cout << "error: " << e.what() << endl; 
		stabilizedFrames.release(); 		
	}
}

void GodStabil::IncodingWithStabilizedImage(Ptr<cv::videostab::IFrameSource> stabilizedFrames, string outputPath, double outputFps)
{	
	VideoWriter writer; 
	Mat stabilizedFrame; 		

	while (!(stabilizedFrame = stabilizedFrames->nextFrame()).empty()) 
	{ 			
		if (!writer.isOpened()) 
		{				
			//writer.open(outputPath, VideoWriter::fourcc('X','V','I','D'), outputFps, stabilizedFrame.size());
			writer.open(outputPath, VideoWriter::fourcc('m','p','e','g'), outputFps, Size(1920, 1080)/*stabilizedFrame.size()*/);
		}				

		Mat SrcImage(stabilizedFrame);
		Mat DstImage;

		resize(SrcImage, DstImage, Size(1920, 1080)/*_outputSize*/, cv::INTER_CUBIC);	
		writer << DstImage; 
	}

}

double GodStabil::Round(double n)
{
	if (n >= 0)
	{
		//소수점이 0.5이상이면 올림
		if (n - (int)n >= 0.5)
			return ++n;
		//아니면 내림
		else
			return n;
	}

	//음수일 경우
	else
	{
		//소수점이 -0.5이상이면 올림
		if (n - (int)n >= -0.5)
			return n;
		//아니면 내림
		else
			return --n;
	}
}

void GodStabil::getAdjustMoveCoordination()
{
	while(1)
	{	
		Mat image;	

		vector<Mat> testMotion = twoPassStabilizer->stabilizationMotions_;

		for(int i = 0; i < testImage.size(); i++)
		{
			image = testImage[i].clone();

			if(image.empty())
				continue;

			int nX = 0, nY = 0;
			if(i == 0)
			{			
				//vc >> image;
				nX = Round(testMotion[i].at<float>(0, 2));
				nY = Round(testMotion[i].at<float>(1, 2));
			}
			else
			{	
				nX = Round(testMotion[i].at<float>(0, 2)) - _subPoint[i].x*_samplingScale;
				nY = Round(testMotion[i].at<float>(1, 2)) - _subPoint[i].y*_samplingScale;
				cout <<testMotion[i]<<endl;
			}

			cout << "x : " << nX << " y : " << nY << endl;

			//Mat original;
			//resize(image, original, Size(image.cols/2, image.rows/2));
			//imshow("original",original);

			moveImage(&image, nX, nY);

			resize(image, image, Size(image.cols/2, image.rows/2));
			//imshow("test",image);
			//waitKey(30);
		}	
	}
	_subPoint.clear();
}

void GodStabil::moveImage(Mat* gMat, int nX, int nY)
{
	Mat gMatCut, gMatPaste;
	int nCutTop,nCutHeight,nCutLeft,nCutWidth;
	int nPasteTop=0,nPasteBottom=0,nPasteLeft=0,nPasteRight=0;

	if ( nX > 0 )
	{
		nCutLeft = 0;
		nCutWidth = gMat->cols - nX;

		nPasteLeft = nX;
	}
	else
	{
		nCutLeft = -nX;
		nCutWidth = gMat->cols + nX;

		nPasteRight = -nX;
	}

	if ( nY > 0 )
	{
		nCutTop = 0;
		nCutHeight = gMat->rows - nY;

		nPasteTop = nY;
	}
	else
	{
		nCutTop = -nY;
		nCutHeight = gMat->rows + nY;

		nPasteBottom = -nY;
	}

	gMatCut = (*gMat)(cv::Rect(nCutLeft, nCutTop,nCutWidth, nCutHeight));
	copyMakeBorder(gMatCut, gMatPaste, nPasteTop,nPasteBottom,nPasteLeft,nPasteRight, BORDER_CONSTANT, 0);
	gMatPaste.copyTo(*gMat);	
	
	
}

IMAGE_SIZE getImageSize(int _width, int _height)
{
	IMAGE_SIZE image_Size;

	if (_width > 1920 && _height > 1080)
		image_Size = UHD_SIZE;
	else if (_width > 1080 && _height > 720)
		image_Size = FHD_SIZE;
	else
		image_Size = ETC_SIZE;

	return image_Size;
}
