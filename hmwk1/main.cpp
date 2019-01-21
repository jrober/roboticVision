/*
 * main.cpp
 *
 *  Created on: Jan 17, 2019
 *      Author: justin
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
using namespace cv;

// harris corners
void corners(Mat &input, Mat &output){

	Mat grayScale;
	int thresh = 150; //max 255
	int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;

    // create a new matrix
	Mat dst = Mat::zeros( input.size(), CV_32FC1 );

    cvtColor( input, grayScale, CV_BGR2GRAY );
   
    cornerHarris(grayScale, dst, blockSize, apertureSize, k );
    
    
    Mat dst_norm, dst_norm_scaled;
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );

    
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > thresh )
            {
                circle( dst_norm_scaled, Point(j,i), 5,  Scalar(0), 2, 8, 0 );
            }
        }
    }

	output = dst_norm_scaled;
	cvtColor(output,output,COLOR_GRAY2BGR);
}

// Threshold
void threshold(Mat &input,Mat &output){
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
	threshold(grayScale, output, 100, 255,0);
	cvtColor(output,output,COLOR_GRAY2BGR);
}

// Canny Edges
void canny(Mat &input, Mat &output){
	Mat dst;
	Mat grayScale;
	Mat detected_edges;

	int ratio = 3;
	int kernel_size = 3;
	int lowThreshold = 50;

	// Create a matrix of the same type and size as src (for dst)
 	dst.create( input.size(), input.type() );

  	// Convert the image to grayscale
  	cvtColor( input, grayScale, CV_BGR2GRAY );

	/// Reduce noise with a kernel 3x3
	blur( grayScale, detected_edges, Size(3,3) );

	/// Canny detector
	Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);

	input.copyTo( dst, detected_edges);
	output = detected_edges;
	cvtColor(output,output,COLOR_GRAY2BGR);
	//output = dst // Use to maintain color of the lines;
}


int main( int argc, char** argv )
{
	char keyPress;
	VideoCapture video(0); // get a camera object
	Mat frame; // allocate an image buffer object
	Mat outFrame;
	Mat grayScale;
	namedWindow("Roberts", CV_WINDOW_AUTOSIZE); // initialize a display window

	//imshow("Roberts", frame); // display the grabbed frame
	//keyPress = (char)waitKey(0);

	// Default resolution of the frame is obtained.The default resolution is system dependent.
	int frame_width = video.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = video.get(CV_CAP_PROP_FRAME_HEIGHT);


	// Create a video write object.
	VideoWriter VOut;


	// Initialize video write object (only done once). Change frame size to match your camera resolution.
	VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(frame_width, frame_height), 1);
	//VOut.open("VideoOut.avi", -1 , 30, Size(frame_width, frame_height), 1); // use this if you don’t have the correct codec



	for(int i = 0; i < 1000; i++){
		// get frame from video
		video >> frame;

		keyPress = (char)waitKey(40);
		//t = Threshold
		if(keyPress == 't'){
			threshold(frame,outFrame);
		}else if(keyPress == 'e'){
			canny(frame,outFrame);
		}else if(keyPress == 'c'){
			corners(frame,outFrame);
		}
		else{
			outFrame = frame;
		}
		
		imshow("Roberts", outFrame);
		waitKey(5);
		//VOut << outFrame;

	}



	// When everything done, release the video capture and write object
	video.release();
	VOut.release();

	// Closes all the windows
	destroyAllWindows();

	return 0;
}




