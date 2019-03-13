#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <fstream>
#include <sstream>

using namespace cv;

// get rectangle center
Point rectCenter(Rect2d r){

	return Point(r.x + r.width/2,r.y + r.height/2);
}

// Threshold
void thresholder(Mat &input,Mat &output){
	// Convert the image to Gray
	Mat grayScale;
	cvtColor( input, grayScale, CV_BGR2GRAY );

	/*
		 0: Binary
		 1: Binary Inverted
		 2: Threshold Truncated
		 3: Threshold to Zero
		 4: Threshold to Zero Inverted
	*/
	threshold(grayScale, output, 20, 255,0);
	//cvtColor(output,output,COLOR_GRAY2BGR);
}

void difference(Mat prevInput,Mat currentInput, Mat &output){

	absdiff(prevInput, currentInput, output);
	thresholder(output, output);
	//erode(output, output, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
}

bool inBounds(Rect2d rectange, int rows, int cols){
	if(rectange.x <= 0)
	{
		return false;
	}
	if(rectange.x + rectange.width >= cols)
	{
		return false;
	}
	if(rectange.y <= 0)
	{
		return false;
	}
	if(rectange.y + rectange.height >= rows)
	{
		return false;
	}
}

int main( int argc, char** argv )
{
  // outer chessboard corners
  std::vector<Point2f> outerCornersL;
	std::vector<Point2f> outerCornersR;
  std::vector<Point2f> newOuterCornersL;
  std::vector<Point2f> newOuterCornersR;
  std::vector<Point3f> transformedL;
  std::vector<Point3f> transformedR;
  std::vector<Point3f> pixelL;
  std::vector<Point3f> pixelR;
	std::vector<Point3f> transformedL2;
  std::vector<Point3f> transformedR2;


  // chessboard corners
  std::vector<Point2f> cornersLeft;
  std::vector<Point2f> cornersRight;
	// std::vector<std::vector<Point3f>> objectPoints;
	// float multiplier = 3.88;
	Mat frame;
	Mat grayScale;
	Size patternsize(10,7);
	Mat outFrame;
	std::stringstream ss;
	bool firstTime = true;
	Size imageSize;

	// initialize a display window
	namedWindow("Left", CV_WINDOW_AUTOSIZE);
	namedWindow("Right", CV_WINDOW_AUTOSIZE);
	namedWindow("CroppedLeft", CV_WINDOW_AUTOSIZE);
	namedWindow("CroppedRight", CV_WINDOW_AUTOSIZE);


  Mat cameraLeft, cameraRight, distortionLeft, distortionRight;
	Point plCurrent;
	Point prCurrent;
	Rect2d croppedLeftRect;
	Rect2d croppedRightRect;

  FileStorage fin("/home/justin/Documents/school/roboticVision/JustinsRepo/left_cam.yaml", cv::FileStorage::READ);
  fin["mtx"] >> cameraLeft;
  fin["dist"] >> distortionLeft;
  fin.release();

  FileStorage fin2("/home/justin/Documents/school/roboticVision/JustinsRepo/right_cam.yaml", cv::FileStorage::READ);
  fin2["mtx"] >> cameraRight;
  fin2["dist"] >> distortionRight;
  fin2.release();

  FileStorage fin3("/home/justin/Documents/school/roboticVision/JustinsRepo/stereo.yaml", cv::FileStorage::READ);
  cv::Mat rotation, translation,
   essential, fundamental;
  fin3["R"] >> rotation;
  fin3["E"] >> essential;
  fin3["T"] >> translation;
  fin3["F"] >> fundamental;
  fin3.release();


  ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3b/images/stereo/stereoL26.bmp";
	Mat leftStereo = imread(ss.str());
	ss.str("");

	ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk3b/images/stereo/stereoR26.bmp";
	Mat rightStereo = imread(ss.str());
	ss.str("");

	if(firstTime){
		firstTime = false;
		imageSize = leftStereo.size();
		//std::cout << imageSize << std::flush;
	}

  //Output Matricies
  Mat R1;
  Mat R2;
  Mat P1;
  Mat P2;
  Mat Q;

  stereoRectify(cameraLeft, distortionLeft, cameraRight, distortionRight, imageSize, rotation, translation,
    R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY);


  std::cout << "image size = " << std::endl << " " << imageSize << std::endl << std::endl;
  std::cout << "Camera Left = " << std::endl << " " << cameraLeft << std::endl << std::endl;
  std::cout << "Distortion Left = " << std::endl << " " << distortionLeft << std::endl << std::endl;
  std::cout << "Camera Right = " << std::endl << " " << cameraRight << std::endl << std::endl;
  std::cout << "Distortion Right = " << std::endl << " " << distortionRight << std::endl << std::endl;
  std::cout << "rotation = " << std::endl << " " << rotation << std::endl << std::endl;
  std::cout << "translation = " << std::endl << " " << translation << std::endl << std::endl;
  // std::cout << "m = " << std::endl << " " << m << std::endl << std::endl;
  std::cout << "essential = " << std::endl << " " << essential << std::endl << std::endl;
  std::cout << "fundamental = " << std::endl << " " << fundamental << std::endl << std::endl;
  std::cout << "R1 = " << std::endl << " " << R1 << std::endl << std::endl;
  std::cout << "R2 = " << std::endl << " " << R2 << std::endl << std::endl;
  std::cout << "P1 = " << std::endl << " " << P1 << std::endl << std::endl;
  std::cout << "P2 = " << std::endl << " " << P2 << std::endl << std::endl;
  std::cout << "Q = " << std::endl << " " << Q << std::endl << std::endl;

  // Left Stereo Corners
	cvtColor( leftStereo, grayScale, CV_BGR2GRAY );

	bool patternfound = findChessboardCorners(grayScale, patternsize, cornersLeft,
	        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
	        + CALIB_CB_FAST_CHECK);

	if(patternfound)
	  cornerSubPix(grayScale, cornersLeft, Size(11, 11), Size(-1, -1),
	    TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


  outerCornersL.push_back(cornersLeft[0]);
  outerCornersL.push_back(cornersLeft[9]);
  outerCornersL.push_back(cornersLeft[(10 * 6) + 1 - 1]);
  outerCornersL.push_back(cornersLeft[(10 * 7) - 1]);

  // // Draw the outer corners before distortion and rectification
  // for(int i = 0; i < outerCornersL.size(); i++){
	// 	//circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
	// 	circle(leftStereo, outerCornersL[i], 6, Scalar(255,0,0), 2, 8, 0);
	// }

//	drawChessboardCorners(leftStereo, patternsize, Mat(cornersLeft), patternfound);




  // Right Stereo Corners
	cvtColor(rightStereo, grayScale, CV_BGR2GRAY );

	patternfound = findChessboardCorners(grayScale, patternsize, cornersRight,
	        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
	        + CALIB_CB_FAST_CHECK);

	if(patternfound)
	  cornerSubPix(grayScale, cornersRight, Size(11, 11), Size(-1, -1),
	    TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

  outerCornersR.push_back(cornersRight[0]);
  outerCornersR.push_back(cornersRight[9]);
  outerCornersR.push_back(cornersRight[(10 * 6) + 1 - 1]);
  outerCornersR.push_back(cornersRight[(10 * 7) - 1]);

  // Draw the outer corners before distortion and rectification
  // for(int i = 0; i < outerCornersR.size(); i++){
	// 	//circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
	// 	circle(rightStereo, outerCornersR[i], 6, Scalar(255,0,0), 2, 8, 0);
	// }

	// drawChessboardCorners(rightStereo, patternsize, Mat(cornersRight), patternfound);

  // undistortPoints(OldPoints, NewPoints, camMatrix,camDistort, R1/R2, P1/P2);

  undistortPoints(outerCornersL, newOuterCornersL, cameraLeft, distortionLeft, R1, P1 );

  // Draw the outer corners before distortion and rectification
  for(int i = 0; i < newOuterCornersL.size(); i++){
    //circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    circle(leftStereo, outerCornersL[i], 6, Scalar(255,0,0), 2, 8, 0);
  }

  undistortPoints(outerCornersR, newOuterCornersR, cameraRight, distortionRight, R2, P2 );

  // Draw the outer corners before distortion and rectification
  for(int i = 0; i < newOuterCornersR.size(); i++){
    //circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    circle(rightStereo, outerCornersR[i], 6, Scalar(255,0,0), 2, 8, 0);
  }


	imshow("Right", rightStereo);
  imshow("Left", leftStereo);

  imwrite("rightStereo.jpg", rightStereo);
  imwrite("leftStereo.jpg", leftStereo);



  waitKey(0);

  for(int i = 0; i < newOuterCornersL.size(); i++)
  {
    float diff = newOuterCornersR[i].x - newOuterCornersL[i].x;
    pixelL.push_back(Point3f(newOuterCornersL[i].x,newOuterCornersL[i].y,-diff));
    pixelR.push_back(Point3f(newOuterCornersR[i].x,newOuterCornersR[i].y,-diff));
  }

  perspectiveTransform(pixelR, transformedR, Q);
  perspectiveTransform(pixelL, transformedL, Q);

  std::cout << "transformed R" << std::endl << transformedR << std::endl;
  std::cout << "transformed L" << std::endl << transformedL << std::endl;



  // initialize region of interest
  ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk4/images/baseballCatcher/first/PL51.bmp";
	Mat leftShot1 = imread(ss.str());
	ss.str("");

	ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk4/images/baseballCatcher/first/PR51.bmp";
	Mat rightShot1 = imread(ss.str());
	ss.str("");

  // Select ROI
  Rect2d rl = selectROI(leftShot1);

  // Crop image
  Mat firstLeftCropped = leftShot1(rl);

  // Select ROI
  Rect2d rr = selectROI(rightShot1);

  // Crop image
  Mat firstRightCropped = rightShot1(rr);

  // create a tracker object
  Ptr<Tracker> trackerLeft = TrackerMOSSE::create();
  Ptr<Tracker> trackerRight = TrackerMOSSE::create();

  // initialize the tracker
  trackerLeft->init(leftShot1,rl);
  trackerRight->init(rightShot1,rr);

	// initialize ball position
	plCurrent = rectCenter(rl);
	prCurrent = rectCenter(rr);

	croppedLeftRect = rl;
	croppedLeftRect = rr;

	Mat prevLeft = leftShot1;
	Mat prevRight = rightShot1;

  //Find Baseball Location in each image
  for(int i = 51; i < 100; i++){

    //Left image
    ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk4/images/baseballCatcher/first/PL" << i << ".bmp";
    Mat leftShot = imread(ss.str());
    ss.str("");

		// Right Image
    ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk4/images/baseballCatcher/first/PR" << i << ".bmp";
    Mat rightShot = imread(ss.str());
    ss.str("");

		// update the tracking result
    bool trackedLeft = trackerLeft->update(leftShot,rl);

		// update the tracking result
    bool trackedRight = trackerRight->update(rightShot,rr);


    //Crop
		Mat croppedLeft;
    difference(leftShot1,leftShot,croppedLeft);

		if(inBounds(rl,leftShot.rows,leftShot.cols)){
			croppedLeftRect = rl;
		}

		croppedLeft = croppedLeft(croppedLeftRect);

    // erode out the noise
    erode(croppedLeft, croppedLeft, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));

    // dilate again to fill in holes
    dilate(croppedLeft, croppedLeft, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));

    // find moments of the image
    Moments m = moments(croppedLeft,true);
    Point pl(m.m10/m.m00, m.m01/m.m00);


		// circle( croppedLeft, pl,
    // 20,
    // Scalar(0,
    // 0,255),
    // 2, 8, 0 );

		imshow("CroppedLeft",croppedLeft);

		pl.x = pl.x + croppedLeftRect.x;
		pl.y = pl.y + croppedLeftRect.y;

		// shift coordinates by cropped amount
		if(pl.x > 0 && pl.x < leftShot.cols && pl.y > 0 && pl.y < leftShot.rows){
			plCurrent.x = pl.x;
			plCurrent.y = pl.y;
		}


    // coordinates of centroid
    std::cout<< Mat(pl)<< std::flush;



		//Crop
		Mat croppedRight;
		difference(rightShot1,rightShot,croppedRight);

		if(inBounds(rr,rightShot.rows,rightShot.cols)){
			croppedRightRect = rr;
		}

		croppedRight = croppedRight(croppedRightRect);

    // erode out the noise
    erode(croppedRight,croppedRight, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));

    // dilate again to fill in holes
    dilate(croppedRight,croppedRight, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));

    // find moments of the image
    m = moments(croppedRight,true);
    Point pr(m.m10/m.m00, m.m01/m.m00);

		// circle( croppedRight, pr,
    // 20,
    // Scalar(0,
    // 0,255),
    // 2, 8, 0 );

		imshow("CroppedRight",croppedRight);

		// shift coordinates by cropped amount
		pr.x = pr.x + croppedRightRect.x;
		pr.y = pr.y + croppedRightRect.y;

		// shift coordinates by cropped amount
		if(pr.x > 0 && pr.x < rightShot.cols && pr.y > 0 && pr.y < rightShot.rows){
			prCurrent.x = pr.x;
			prCurrent.y = pr.y;
		}

    // coordinates of centroid
    std::cout<< Mat(pr)<< std::flush;

    // cvtColor( rightShot, rightShot, COLOR_GRAY2BGR );
    // cv2.circle(img, center, radius, color, thickness=1, lineType=8, shift=0)

    // draw the tracked object
    rectangle( leftShot, rl, Scalar( 255, 0, 0 ), 2, 1 );

		circle( leftShot, rectCenter(rl),
    15,
    Scalar(0,
    0,255),
    2, 8, 0 );

    // show image with the tracked object
    imshow("Left",leftShot);

    // draw the tracked object
    rectangle( rightShot, rr, Scalar( 255, 0, 0 ), 2, 1 );

		circle( rightShot, rectCenter(rr),
    15,
    Scalar(0,
    0,255),
    2, 8, 0 );

    // show image with the tracked object
    imshow("Right",rightShot);

    ss << "left" << i <<".jpg";
    imwrite(ss.str(), leftShot);
    ss.str("");

    ss << "right" << i <<".jpg";
    imwrite(ss.str(), rightShot);
    ss.str("");

		Point rightLocation = rectCenter(rr);
		Point leftLocation = rectCenter(rl);



		if(trackedLeft || trackedRight){
			std::vector<Point3f> positionLXYZ;
			std::vector<Point2f> pixelLoriginal;
			std::vector<Point2f> pixelLundistorted;

			std::vector<Point3f> positionRXYZ;
			std::vector<Point2f> pixelRoriginal;
			std::vector<Point2f> pixelRundistorted;

			std::vector<Point3f> pixelLMat;
			std::vector<Point3f> pixelRMat;

			pixelLoriginal.push_back(Point2f(leftLocation.x,leftLocation.y));

			pixelRoriginal.push_back(Point2f(rightLocation.x,rightLocation.y));

			undistortPoints(pixelLoriginal, pixelLundistorted, cameraLeft, distortionLeft, R1, P1 );

			undistortPoints(pixelRoriginal, pixelRundistorted, cameraRight, distortionRight, R2, P2 );

			float diff = pixelRundistorted[0].x - pixelLundistorted[0].x;

	    pixelLMat.push_back(Point3f(pixelLundistorted[0].x,pixelLundistorted[0].y,-diff));

			pixelRMat.push_back(Point3f(pixelRundistorted[0].x,pixelRundistorted[0].y,-diff));

		  perspectiveTransform(pixelLMat, positionLXYZ, Q);
			transformedL2.push_back(positionLXYZ[0]);

			perspectiveTransform(pixelRMat, positionRXYZ, Q);
			transformedR2.push_back(positionRXYZ[0]);

			std::cout << Mat(positionLXYZ[0]) << " left camera location \n";
			std::cout << Mat(positionRXYZ[0]) << " right camera location \n";
		}

    waitKey(0);

		prevRight = rightShot;
		prevLeft = leftShot;


  }

	waitKey(0);
	return 0;
}





































//Calibrations Parameters
// Mat cameraLeft = (Mat_<double>(3,3) << 1678.435592347326, 0, 342.9566750991285,
//  0, 1681.497378894579, 283.8306476045657,
//  0, 0, 1);
//
//
// Mat distortionLeft = (Mat_<double>(1,5) << -0.4099022197771052, -3.07245286106378, -0.001219386650755467,
// -0.003035720838675977, 41.52599117247595);
//
//
// Mat cameraRight = (Mat_<double>(3,3) << 1710.102370081143, 0, 315.1291187789068,
//  0, 1713.964351741807, 250.845819849757,
//  0, 0, 1);
//
//
// Mat distortionRight = (Mat_<double>(1,5) << -0.4788104510406055, -0.9411823233204554, 0.0006441849284121036,
// 0.003574448062106206, 25.35880156931097);



// Mat rotation = (Mat_<double>(3,3) << 0.99989629564382, -0.0002872074927716673, 0.01439845372332044,
//  1.231731930994076e-05, 0.9998178055476596, 0.01908809990925838,
//  -0.01440131265024648, -0.01908594303979452, 0.9997141236234645);
//
//
// Mat translation = (Mat_<double>(3,1) << -20.30818326354131,-0.0211919627040026,
//  4.674500475756695);
//
//
// Mat m = (Mat_<double>(4,4) <<  0.99989629564382, -0.0002872074927716673, 0.01439845372332044,-20.30818326354131,
//  1.231731930994076e-05, 0.9998178055476596, 0.01908809990925838,-0.0211919627040026,
//  -0.01440131265024648, -0.01908594303979452, 0.9997141236234645, 4.674500475756695,
//   0,   0,   0,   1);
//
//
// Mat essential = (Mat_<double>(3,3) << 0.0002476147655983392, -4.67324433910948, -0.1104132365296127,
//  4.381551213157632, -0.3889433805712597, 20.36968321247573,
//  0.02093962262729236, -20.30448931170405, -0.3873394996157055);
//
//
// Mat fundamental = (Mat_<double>(3,3) << 1.693748260424144e-09, -3.190797902107096e-05, 0.007788235017331002,
//  2.990339790318582e-05, -2.649643943957106e-06, 0.223832401920846,
//  -0.007256734099548239, -0.2263602530127546, 1);
//
//
// Mat R1 = (Mat_<double>(3,3) << 0.9776460852346666, 0.005018067580897621, -0.2101974096488459,
//  -0.003013048640466542, 0.9999468702276076, 0.009857903423249034,
//  0.2102357095338976, -0.009004205670988792, 0.9776092628023801);
//
//
// Mat R2 = (Mat_<double>(3,3) << 0.974516740174627, 0.00101692613978341, -0.2243124806617875,
//  -0.003157989841451422, 0.9999528169879738, -0.00918645187094019,
//  0.2242925549802754, 0.009660727666280431, 0.9744739709819745);
//
//
// Mat P1 = (Mat_<double>(3,4) << 1642.493835988404, 0, 710.0826411247253, 0,
//  0, 1642.493835988404, 267.7925643920898, 0,
//  0, 0, 1, 0);
//
//
// Mat P2 = (Mat_<double>(3,4) << 1642.493835988404, 0, 710.0826411247253, -34228.31487175097,
//  0, 1642.493835988404, 267.7925643920898, 0,
//  0, 0, 1, 0);
//
//
// Mat Q = (Mat_<double>(4,4) << 1, 0, 0, -710.0826411247253,
//  0, 1, 0, -267.792564392089,
//  0, 0, 0, 1642.493835988404,
//  0, 0, 0.04798640663855681, -0);
//





/*
camera matrix left
[1678.435592347326, 0, 342.9566750991285;
0, 1681.497378894579, 283.8306476045657;
0, 0, 1]
distortion matrix left
[-0.4099022197771052, -3.07245286106378, -0.001219386650755467, -0.003035720838675977, 41.52599117247595]
camera matrix right
[1710.102370081143, 0, 315.1291187789068;
0, 1713.964351741807, 250.845819849757;
0, 0, 1]
distortion matrix right
[-0.4788104510406055, -0.9411823233204554, 0.0006441849284121036, 0.003574448062106206, 25.35880156931097]
Rotation
[0.99989629564382, -0.0002872074927716673, 0.01439845372332044;
1.231731930994076e-05, 0.9998178055476596, 0.01908809990925838;
-0.01440131265024648, -0.01908594303979452, 0.9997141236234645]
Translation
[-20.30818326354131;
-0.0211919627040026;
4.674500475756695]
Esential
[0.0002476147655983392, -4.67324433910948, -0.1104132365296127;
4.381551213157632, -0.3889433805712597, 20.36968321247573;
0.02093962262729236, -20.30448931170405, -0.3873394996157055]
Fundamental
[1.693748260424144e-09, -3.190797902107096e-05, 0.007788235017331002;
2.990339790318582e-05, -2.649643943957106e-06, 0.223832401920846;
-0.007256734099548239, -0.2263602530127546, 1]
right lines[0.011239073, -0.99993682, 327.55652;
0.0082926452, -0.99996561, 305.5726;
0.0053709978, -0.99998558, 283.77148]
left lines[-0.021201197, 0.99977523, -393.85349;
-0.018143026, 0.99983537, -372.11124;
-0.015075772, 0.99988633, -350.30212]


R1[0.9776460852346666, 0.005018067580897621, -0.2101974096488459;
-0.003013048640466542, 0.9999468702276076, 0.009857903423249034;
0.2102357095338976, -0.009004205670988792, 0.9776092628023801]
R2[0.974516740174627, 0.00101692613978341, -0.2243124806617875;
-0.003157989841451422, 0.9999528169879738, -0.00918645187094019;
0.2242925549802754, 0.009660727666280431, 0.9744739709819745]
P1[1642.493835988404, 0, 710.0826411247253, 0;
0, 1642.493835988404, 267.7925643920898, 0;
0, 0, 1, 0]
P2[1642.493835988404, 0, 710.0826411247253, -34228.31487175097;
0, 1642.493835988404, 267.7925643920898, 0;
0, 0, 1, 0]
Q[1, 0, 0, -710.0826411247253;
0, 1, 0, -267.7925643920898;
0, 0, 0, 1642.493835988404;
0, 0, 0.04798640663855681, -0]

*/
