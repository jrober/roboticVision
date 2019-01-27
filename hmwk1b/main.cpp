/*
 * main.cpp
 *
 *  Created on: Jan 17, 2019
 *      Author: justin
 */

	//std::cout << ss.str() << std::flush;

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
using namespace cv;


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
	threshold(grayScale, output, 10, 255,0);
	//cvtColor(output,output,COLOR_GRAY2BGR);
}

void difference(Mat prevInput,Mat currentInput, Mat &output){

	absdiff(prevInput, currentInput, output);
	thresholder(output, output);
	//erode(output, output, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
}



int main( int argc, char** argv )
{
	Mat initialLframe;
	Mat initialRframe;
	Mat frame; // allocate an image buffer object
	Mat processFrame;
	Moments m;
	std::stringstream ss;
	char leftRight = 'L';
	
 	// initialize a display window
	namedWindow("Roberts", CV_WINDOW_AUTOSIZE);

	//initialize frames
	initialLframe = imread("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk1b/images/1L05.jpg"); 
	initialRframe = imread("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk1b/images/1R05.jpg"); 


	// Default resolution of the frame is obtained.The default resolution is system dependent.
	int frame_width = initialLframe.cols;
	int frame_height = initialLframe.rows;

	// Create a video write object.
	VideoWriter VOut;

	// Initialize video write object (only done once). Change frame size to match your camera resolution.
	VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(frame_width, frame_height), 1);
	//VOut.open("VideoOut.avi", -1 , 30, Size(frame_width, frame_height), 1); // use this if you don’t have the correct codec

	//initializeFrameNumber
	int frameNum = 5;

	//Loop through each image
	for(int i = 0; i < 36; i++){

		if(frameNum < 10){
			ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk1b/images/1" << leftRight << 0 << frameNum << ".jpg";
		}else{
			ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk1b/images/1" << leftRight << frameNum << ".jpg";
		}

		frame = imread(ss.str());
		ss.str("");

		if(leftRight == 'L'){
			difference(initialLframe,frame,processFrame);
		}else{
			difference(initialRframe,frame,processFrame);
		}
		
		// erode out the noise
		erode(processFrame, processFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));
		
		// dilate again to fill in holes
		dilate(processFrame, processFrame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));
		
		// find moments of the image
		m = moments(processFrame,true);
		Point p(m.m10/m.m00, m.m01/m.m00);
		 
		// coordinates of centroid
		std::cout<< Mat(p)<< std::flush;

		// cv2.circle(img, center, radius, color, thickness=1, lineType=8, shift=0)
		circle( frame, p, 
		15, 
		Scalar(255, 
		0,0), 
		2, 8, 0 );

		imshow("Roberts", processFrame);
		waitKey(100);

		//cvtColor(frame,frame,COLOR_GRAY2BGR);
		VOut << frame;	
		//cvtColor(processFrame,processFrame,COLOR_GRAY2BGR);
		//VOut << processFrame;	

		frameNum++;
	}

	// When everything done, release the video capture and write object
	VOut.release();
	// Closes all the windows
	//destroyAllWindows();

	return 0;
}