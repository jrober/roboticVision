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
	float multiplier = 3.88;
	Mat frame;
	Mat grayScale;
	Size patternsize(10,7);
	Mat outFrame;
	std::stringstream ss;
	bool firstTime = true;
	Size imageSize;
	int skip = 3;

	// initialize a display window
	namedWindow("Roberts", CV_WINDOW_AUTOSIZE);
	namedWindow("Second", CV_WINDOW_AUTOSIZE);

	std::vector<Point3f> temp;
	for(double i = 0; i < 7; i++){

		for(double j = 0; j < 10; j++){
			temp.push_back(Point3f(j*multiplier,i*multiplier,0));
		}
	}


	for(int i = 1; i <= 99; i++){
		if(i % skip != 0)
			continue;
		objectPoints.push_back(temp);
	}



	//Loop through each left image
	for(int i = 1; i <= 99; i++){
		if(i % skip != 0)
			continue;
		ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3b/images/left/leftL" << i << ".bmp";
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
		// waitKey(0);
	}

	firstTime = true;
	//Loop through each right image
	for(int i = 1; i <= 99; i++){
		if(i % skip != 0)
			continue;
		ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3b/images/right/rightR" << i << ".bmp";
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
		// waitKey(0);
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

	for(int i = 1; i <= 99; i++){
		if(i % skip != 0)
			continue;
		ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3b/images/stereo/stereoL" << i << ".bmp";
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
	for(int i = 1; i <= 99; i++){
		if(i % skip != 0)
			continue;
		ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3b/images/stereo/stereoR" << i << ".bmp";
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


	ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3b/images/stereo/stereoL1.bmp";
	Mat leftStereo = imread(ss.str());
	ss.str("");

	ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3b/images/stereo/stereoR1.bmp";
	Mat rightStereo = imread(ss.str());
	ss.str("");

	Mat undistLeft;
	Mat undistRight;

	// undistort images
	undistort(leftStereo, undistLeft, cameraMatrixL, distCoeffsL);

	undistort(rightStereo, undistRight, cameraMatrixR, distCoeffsR);

	imageSize = undistLeft.size();

	std::vector<Point2f> corners; //this will be filled by the detected corners
	std::vector<Point2f> cornersLeft; //this will be used as the selected left points
	std::vector<Point2f> cornersRight; // this will be used as the selected right points


	//////////Choose three left corners//////////////
	cvtColor( undistLeft, grayScale, CV_BGR2GRAY );

	//CALIB_CB_FAST_CHECK saves a lot of time on images
	//that do not contain any chessboard corners
	bool patternfound = findChessboardCorners(grayScale, patternsize, corners,
	    CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
	    + CALIB_CB_FAST_CHECK);

	if(patternfound)
		cornerSubPix(grayScale, corners, Size(11, 11), Size(-1, -1),
			TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	// choose three corners on left image
	cornersLeft.push_back(corners[0]);
	cornersLeft.push_back(corners[11]);
	cornersLeft.push_back(corners[22]);

	for(int i = 0; i < cornersLeft.size(); i++){
		//circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
		circle(undistLeft, cornersLeft[i], 6, Scalar(255,0,0), 2, 8, 0);
	}



	// imshow("Roberts", undistLeft);
	// waitKey(0);

	//////////Choose three right corners//////////////
	cvtColor( undistRight, grayScale, CV_BGR2GRAY );

	//CALIB_CB_FAST_CHECK saves a lot of time on images
	//that do not contain any chessboard corners
	patternfound = findChessboardCorners(grayScale, patternsize, corners,
	    CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
	    + CALIB_CB_FAST_CHECK);

	if(patternfound)
		cornerSubPix(grayScale, corners, Size(11, 11), Size(-1, -1),
			TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	// choose three corners on left image
	cornersRight.push_back(corners[33]);
	cornersRight.push_back(corners[44]);
	cornersRight.push_back(corners[55]);

	for(int i = 0; i < cornersRight.size(); i++){
		//circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
		circle(undistRight, cornersRight[i], 6, Scalar(255,0,0), 2, 8, 0);
	}

	// imshow("Roberts", undistRight);
	// waitKey(0);

	//computeCorrespondEpilines(InputArray points, int whichImage, InputArray F, OutputArray lines)
	Mat rightLines;
	computeCorrespondEpilines(cornersRight,2,F,rightLines);
	std::cout << "right lines" << rightLines << "\n";

	Mat leftLines;
	computeCorrespondEpilines(cornersLeft,1,F,leftLines);
	std::cout << "left lines" << leftLines << "\n";

	int numCols = undistRight.cols;


	// right lines go on left image
	for(int i = 0; i < rightLines.rows; i++){
		float a = rightLines.at<float>(i,0);
		float b = rightLines.at<float>(i,1);
		float c = rightLines.at<float>(i,2);
		// std::cout << "a: " << a;
		// std::cout << "b: " << b;
		// std::cout << "c: " << c;
		// std::cout << "\n";
		Point2i pt1(0,(int)(-c/b));
		Point2i pt2(numCols,(-a*numCols-c)/b);
		line(undistLeft,pt1,pt2,Scalar(0,0,255),2,1,0);

	}


	// left lines go on the right image
	for(int i = 0; i < leftLines.rows; i++){
		float a = leftLines.at<float>(i,0);
		float b = leftLines.at<float>(i,1);
		float c = leftLines.at<float>(i,2);
		// std::cout << "a: " << a;
		// std::cout << "b: " << b;
		// std::cout << "c: " << c;
		// std::cout << "\n";
		Point2i pt1(0,(int)(-c/b));
		Point2i pt2(numCols,(-a*numCols-c)/b);
		line(undistRight,pt1,pt2,Scalar(0,0,255),2,1,0);
	}

	std::cout << "\n\n";

	imshow("Roberts", undistLeft);
	imwrite( "epiLeft.jpg", undistLeft);
	waitKey(0);

	imshow("Second", undistRight);
	imwrite( "epiRight.jpg", undistRight);
	waitKey(0);


	//// Rectify ////
	//stereoRectify(InputArray cameraMatrix1, InputArray distCoeffs1, InputArray cameraMatrix2,
	//InputArray distCoeffs2, Size imageSize, InputArray R,
	// InputArray T, OutputArray R1, OutputArray R2, OutputArray P1,
	//OutputArray P2, OutputArray Q, int flags=CALIB_ZERO_DISPARITY, double alpha=-1,
	// Size newImageSize=Size(), Rect* validPixROI1=0, Rect* validPixROI2=0 )

	//Output Matricies
	Mat R1;
	Mat R2;
	Mat P1;
	Mat P2;
	Mat Q;
	// Size newImageSize = size();
	stereoRectify(cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, imageSize, R, T,
		R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY);

		std::cout << "R1" << R1 << "\n" << "R2" << R2 << "\n" << "P1" << P1 << "\n" << "P2" << P2 << "\n" << "Q" << Q << "\n";

	//initUndistortRectifyMap(InputArray cameraMatrix, InputArray distCoeffs, InputArray R,
	//InputArray newCameraMatrix, Size size, int m1type, OutputArray map1, OutputArray map2)

	//Left image rectify
	Size size = undistLeft.size();
	Mat map1L;
	Mat map2L;
	initUndistortRectifyMap(cameraMatrixL, distCoeffsL, R1, P1, size, CV_32FC1, map1L, map2L);

	size = undistRight.size();
	Mat map1R;
	Mat map2R;
	initUndistortRectifyMap(cameraMatrixR, distCoeffsR, R2, P2, size, CV_32FC1, map1R, map2R);

	Mat newLeft;
	Mat newRight;
	remap( leftStereo, newLeft, map1L, map2L, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
	remap( rightStereo, newRight, map1R, map2R, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );


	// Diff
	Mat diffImage;

	absdiff(leftStereo,newLeft,diffImage);
	imshow("Roberts", newLeft);
	imwrite( "diffLeft.jpg", diffImage);
	waitKey(0);

	absdiff(rightStereo,newRight,diffImage);
	imshow("Second", newRight);
	imwrite( "diffRight.jpg", diffImage);
	waitKey(0);


	// draw left lines
	numCols = newLeft.cols;
	for(int i = 0; i < newLeft.rows/10; i++){
		Point2i pt1(0,10*i);
		Point2i pt2(numCols,10*i);
		line(newLeft,pt1,pt2,Scalar(0,0,255),1,1,0);
	}

	// draw right lines
	numCols = newRight.cols;
	for(int i = 0; i < newRight.rows/10; i++){
		Point2i pt1(0,10*i);
		Point2i pt2(numCols,10*i);
		line(newRight,pt1,pt2,Scalar(0,0,255),1,1,0);
	}

	imshow("Roberts", newLeft);
	imwrite( "rectifiedLeft.jpg", newLeft);
	waitKey(0);

	imshow("Second", newRight);
	imwrite( "rectifiedRight.jpg", newRight);
	waitKey(0);



	// draw right lines on original image
	numCols = leftStereo.cols;
	for(int i = 0; i < leftStereo.rows/10; i++){
		Point2i pt1(0,10*i);
		Point2i pt2(numCols,10*i);
		line(leftStereo,pt1,pt2,Scalar(0,0,255),1,1,0);
	}

	// draw left lines  on original image
	numCols = rightStereo.cols;
	for(int i = 0; i < rightStereo.rows/10; i++){
		Point2i pt1(0,10*i);
		Point2i pt2(numCols,10*i);
		line(rightStereo,pt1,pt2,Scalar(0,0,255),1,1,0);
	}


	imshow("Second", leftStereo);
	imwrite( "originalRight.jpg", leftStereo);
	waitKey(0);

	imshow("Roberts", rightStereo);
	imwrite( "originalLeft.jpg", rightStereo);
	waitKey(0);


	// line	(	InputOutputArray 	img,
	// Point 	pt1,
	// Point 	pt2,
	// const Scalar & 	color,
	// int 	thickness = 1,
	// int 	lineType = LINE_8,
	// int 	shift = 0
	// )

	waitKey(0);
	return 0;
}
