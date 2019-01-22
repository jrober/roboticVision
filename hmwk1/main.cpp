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




// lines
void drawLines(Mat &input, Mat &output){

	Mat dst, cdst;
    Canny(input, dst, 50, 200, 3); 

    // void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0 )
    cvtColor(dst, cdst, CV_GRAY2BGR); 
 
    std::vector<Vec2f> lines;

    // detect lines
    HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 );
 
    // draw lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
    }

    output = cdst;
    //cvtColor(output,output,COLOR_GRAY2BGR);
}

// corner detection with subpixels
void corners2(Mat &input, Mat &output){

  

  /// Parameters for Shi-Tomasi algorithm
  std::vector<Point2f> corners;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int maxCorners = 50;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;
  Mat grayScale;
  RNG rng(12345);

  

  /// Copy the source image
  Mat copy;
  copy = input.clone();

  cvtColor( input, grayScale, CV_BGR2GRAY );

  /// Apply corner detection
  goodFeaturesToTrack( grayScale,
                       corners,
                       maxCorners,
                       qualityLevel,
                       minDistance,
                       Mat(),
                       blockSize,
                       useHarrisDetector,
                       k );


  /// Draw corners detected
  //std::cout<<"** Number of corners detected: "<<corners.size()<<std::endl;
  int r = 6; //radius
  
  //cv2.circle(img, center, radius, color, thickness=1, lineType=8, shift=0)
  for( int i = 0; i < corners.size(); i++ ){
      	circle( copy, corners[i], 
		r, 
		Scalar(255, 
		255,255), 
		-1, 8, 0 ); 
  }

  output = copy;
  //cvtColor(output,output,COLOR_GRAY2BGR);
}

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
	threshold(grayScale, output, 100, 255,0);
	cvtColor(output,output,COLOR_GRAY2BGR);
}

void difference(Mat prevInput,Mat currentInput, Mat &output){
	Mat grayScale1, grayScale2;

	//cvtColor( currentInput, grayScale1, CV_BGR2GRAY );
	//cvtColor( prevInput, grayScale2, CV_BGR2GRAY );

	absdiff(prevInput, currentInput, output);
	thresholder(output, output);
	//erode(output, output, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
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
	Mat prevFrame;
	
 	// initialize a display window
	namedWindow("Roberts", CV_WINDOW_AUTOSIZE);

	// Default resolution of the frame is obtained.The default resolution is system dependent.
	int frame_width = video.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = video.get(CV_CAP_PROP_FRAME_HEIGHT);

	// Create a video write object.
	VideoWriter VOut;


	// Initialize video write object (only done once). Change frame size to match your camera resolution.
	VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(frame_width, frame_height), 1);
	//VOut.open("VideoOut.avi", -1 , 30, Size(frame_width, frame_height), 1); // use this if you don’t have the correct codec

	//initialize frames
	video >> prevFrame;
	video >> frame;



	for(int i = 0; i < 200; i++){
		

		// wait for image processing key
		keyPress = (char)waitKey(50);
		
		//different image processing options 
		if(keyPress == 't'){
			thresholder(frame,outFrame);
		}else if(keyPress == 'e'){
			canny(frame,outFrame);
		}else if(keyPress == 'c'){
			corners2(frame,outFrame);
		}else if(keyPress == 'l'){
			drawLines(frame,outFrame);
		}else if(keyPress == 'd'){
			difference(prevFrame,frame,outFrame);
		}else{
			outFrame = frame;
		}
		
		

		imshow("Roberts", outFrame);
		waitKey();
		VOut << outFrame;

		// get another frame from video
		frame.copyTo(prevFrame);
		video >> frame;

	}

	// When everything done, release the video capture and write object
	video.release();
	VOut.release();

	// Closes all the windows
	destroyAllWindows();

	return 0;
}




