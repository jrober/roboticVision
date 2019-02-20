#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <sstream>

using namespace cv;

int main( int argc, char** argv )
{
	std::vector<std::vector<Point2f>> imagePointsL;
	std::vector<std::vector<Point2f>> imagePointsR;
	std::vector<std::vector<Point3f>> objectPoints;
	float multiplier = 3.88636/2;
	Mat frame;
	Mat grayScale;
	Size patternsize(10,7);
	Mat outFrame;
	std::stringstream ss;
	bool firstTime = true;
	Size imageSize;

	std::vector<Point3f> temp;
	for(double i = 0; i < 7; i++){
		
		for(double j = 0; j < 10; j++){
			temp.push_back(Point3f(j*multiplier,i*multiplier,0));
		}
	}

	for(int i = 0; i <= 31; i++){
		objectPoints.push_back(temp);
	}

	

	//Loop through each left image
	for(int i = 0; i <= 31; i++){
		
		ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3/images/CameraL" << i << ".bmp";
		frame = imread(ss.str());
		ss.str("");

		if(firstTime){
			firstTime = false;
			imageSize = frame.size();
			//std::cout << imageSize << std::flush;
		}

		cvtColor( frame, grayScale, CV_BGR2GRAY );
		
		std::vector<Point2f> corners; //this will be filled by the detected corners

		//CALIB_CB_FAST_CHECK saves a lot of time on images
		//that do not contain any chessboard corners
		bool patternfound = findChessboardCorners(grayScale, patternsize, corners,
		        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
		        + CALIB_CB_FAST_CHECK);

		if(patternfound)
		  cornerSubPix(grayScale, corners, Size(11, 11), Size(-1, -1),
		    TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		imagePointsL.push_back(corners);

		drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
		//cvtColor( frame, outFrame, CV_GRAY2BGR );
		outFrame = frame;
		imshow("Roberts", outFrame);
		//waitKey(0);
	}

	firstTime = true;
	//Loop through each right image
	for(int i = 0; i <= 31; i++){
		
		ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3/images/CameraR" << i << ".bmp";
		frame = imread(ss.str());
		ss.str("");

		if(firstTime){
			firstTime = false;
			imageSize = frame.size();
			//std::cout << imageSize << std::flush;
		}

		cvtColor( frame, grayScale, CV_BGR2GRAY );
		
		std::vector<Point2f> corners; //this will be filled by the detected corners

		//CALIB_CB_FAST_CHECK saves a lot of time on images
		//that do not contain any chessboard corners
		bool patternfound = findChessboardCorners(grayScale, patternsize, corners,
		        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
		        + CALIB_CB_FAST_CHECK);

		if(patternfound)
		  cornerSubPix(grayScale, corners, Size(11, 11), Size(-1, -1),
		    TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		imagePointsR.push_back(corners);

		drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
		//cvtColor( frame, outFrame, CV_GRAY2BGR );
		outFrame = frame;
		imshow("Roberts", outFrame);
		//waitKey(0);
	}


	//Returns
	Mat cameraMatrixL;
	Mat distCoeffsL;
	std::vector<Mat> rvecsL;
	std::vector<Mat> tvecsL;
	
	Mat cameraMatrixR;
	Mat distCoeffsR;
	std::vector<Mat> rvecsR;
	std::vector<Mat> tvecsR;

	double testL = calibrateCamera(objectPoints, imagePointsL, imageSize, cameraMatrixL, distCoeffsL, rvecsL, tvecsL,0 );

	double testR = calibrateCamera(objectPoints, imagePointsR, imageSize, cameraMatrixR, distCoeffsR, rvecsR, tvecsR,0 );

	/*
	std::cout << "camera matrix\n" << std::flush;
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			std::cout << cameraMatrix[i][j] << " \n" << std::flush; 
		}
		std::cout << end << std::flush;
	}*/

	std::cout << "camera matrix left" << std::endl << cameraMatrixL << std::endl << std::flush; 
	std::cout << "distortion matrix left" << std::endl << distCoeffsL << std::endl << std::flush; 

	std::cout << "camera matrix right" << std::endl << cameraMatrixR << std::endl << std::flush; 
	std::cout << "distortion matrix right" << std::endl << distCoeffsR << std::endl << std::flush; 






	// Stereo Calibration//

	//Loop through each stereo left image

	std::vector<std::vector<Point2f>> stereoPointsL;
	std::vector<std::vector<Point2f>> stereoPointsR;

	for(int i = 0; i <= 31; i++){
		
		ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3/images/StereoL" << i << ".bmp";
		frame = imread(ss.str());
		ss.str("");

		if(firstTime){
			firstTime = false;
			imageSize = frame.size();
			//std::cout << imageSize << std::flush;
		}

		cvtColor( frame, grayScale, CV_BGR2GRAY );
		
		std::vector<Point2f> corners; //this will be filled by the detected corners

		//CALIB_CB_FAST_CHECK saves a lot of time on images
		//that do not contain any chessboard corners
		bool patternfound = findChessboardCorners(grayScale, patternsize, corners,
		        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
		        + CALIB_CB_FAST_CHECK);

		if(patternfound)
		  cornerSubPix(grayScale, corners, Size(11, 11), Size(-1, -1),
		    TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		stereoPointsL.push_back(corners);

		drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
		//cvtColor( frame, outFrame, CV_GRAY2BGR );
		outFrame = frame;
		imshow("Roberts", outFrame);
		waitKey(0);
	}

	firstTime = true;
	//Loop through each stereo right image
	for(int i = 0; i <= 31; i++){
		
		ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3/images/StereoR" << i << ".bmp";
		frame = imread(ss.str());
		ss.str("");

		if(firstTime){
			firstTime = false;
			imageSize = frame.size();
			//std::cout << imageSize << std::flush;
		}

		cvtColor( frame, grayScale, CV_BGR2GRAY );
		
		std::vector<Point2f> corners; //this will be filled by the detected corners

		//CALIB_CB_FAST_CHECK saves a lot of time on images
		//that do not contain any chessboard corners
		bool patternfound = findChessboardCorners(grayScale, patternsize, corners,
		        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
		        + CALIB_CB_FAST_CHECK);

		if(patternfound)
		  cornerSubPix(grayScale, corners, Size(11, 11), Size(-1, -1),
		    TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		stereoPointsR.push_back(corners);

		drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
		//cvtColor( frame, outFrame, CV_GRAY2BGR );
		outFrame = frame;
		imshow("Roberts", outFrame);
		waitKey(0);
	}

	Mat R;
	Mat T;
	Mat E;
	Mat F;

	double stereoTest = stereoCalibrate( objectPoints, stereoPointsL, stereoPointsR,cameraMatrixL,
	 distCoeffsL, cameraMatrixR, distCoeffsR, imageSize, 
	 R, T, E, F, CALIB_FIX_INTRINSIC,
	 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6));

	std::cout << "Rotation " << std::endl << R << std::endl << std::flush; 
	std::cout << "Translation " << std::endl << T << std::endl << std::flush; 
	std::cout << "Esential  " << std::endl << E << std::endl << std::flush; 
	std::cout << "Fundamental " << std::endl << F << std::endl << std::flush; 


	return 0;
}
