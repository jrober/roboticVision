#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <sstream>
using namespace cv;



int main( int argc, char** argv )
{
	char keyPress;
	char TempKey_press;
	//VideoCapture video(0); // get a camera object
	Mat frame; // allocate an image buffer object
	Mat outFrame;
	Mat grayScale;
	Size patternsize(10,7); //interior number of corners column X row

 	// initialize a display window
	namedWindow("Roberts", CV_WINDOW_AUTOSIZE);
	

	// wait for image processing key
	keyPress = (char)waitKey(0);

	//task 1
	if(keyPress == '1'){

		std::stringstream ss;

		ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk2/task1/images/AR" << 1 << ".jpg";

		frame = imread(ss.str());
		ss.str("");

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

		drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
		//cvtColor( frame, outFrame, CV_GRAY2BGR );
		outFrame = frame;
		imshow("Roberts", outFrame);
		waitKey(0);
	

		imwrite( "Image.jpg", outFrame);
		
	}
	//task 2
	else if(keyPress == '2'){



		std::vector<std::vector<Point2f>> imagePoints;
		std::vector<std::vector<Point3f>> objectPoints;

		std::vector<Point3f> temp;
		for(int i = 0; i < 7; i++){
			
			for(int j = 0; j < 10; j++){
				temp.push_back(Point3f(j,i,0));
			}
		}

		for(int i = 1; i <= 40; i++){
			objectPoints.push_back(temp);
		}

		std::stringstream ss;

		bool firstTime = true;
		Size imageSize;

		//Loop through each image
		for(int i = 1; i <= 40; i++){
			
			if(firstTime){
				ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk2/task1/images/AR" << i << ".jpg";

				frame = imread(ss.str());
				ss.str("");
				firstTime = false;
				imageSize = frame.size();
				//std::cout << imageSize << std::flush;
			}

			ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk2/task1/images/AR" << i << ".jpg";

			frame = imread(ss.str());
			ss.str("");

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

			imagePoints.push_back(corners);

			drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
			//cvtColor( frame, outFrame, CV_GRAY2BGR );
			outFrame = frame;
			imshow("Roberts", outFrame);
			//waitKey(0);
		}


		//Returns
		Mat cameraMatrix;
		Mat distCoeffs;
		std::vector<Mat> rvecs;
		std::vector<Mat> tvecs;

		/*
		double test = calibrateCamera( const std::vector<std::vector<Point3f>> &objectPoints,...
		 const std::vector<std::vector<Point2f> > &imagePoints, Size imageSize, ...
		 Mat& cameraMatrix, Mat& distCoeffs, std::vector<Mat>& rvecs, std::vector<Mat>& tvecs, int flags=0 );
		*/
		
		double test = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,0 );

		/*
		std::cout << "camera matrix\n" << std::flush;
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				std::cout << cameraMatrix[i][j] << " \n" << std::flush; 
			}
			std::cout << end << std::flush;
		}*/

		std::cout << "camera matrix" << std::endl << cameraMatrix << std::endl << std::flush; 
		std::cout << "distortion matrix" << std::endl << distCoeffs << std::endl << std::flush; 

		//pixel size 4.7µm´4.7µm sx = sy
		double pixelSize = 1/135.0;// 4.7 * .001; //   mm/pixel
		std::cout << "focal length = " << pixelSize * (cameraMatrix.at<double>(0,0)) << std::endl << std::flush; 
	}
	//task 3
	else if(keyPress == '3'){
		Mat intrinsic = Mat::zeros(3,3,CV_32F);
		Mat distortion = Mat::zeros(1,5,CV_32F);
		Mat newcameramtx;
		Mat diffImage;
		Mat close = imread("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk2/task3/Close.jpg");
		Mat far = imread("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk2/task3/Far.jpg");
		Mat turned = imread("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk2/task3/Turned.jpg");
		
		//open file stream
		std::ifstream myfile;
  		myfile.open("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk2/task3/matricies.txt");

  		//load intrinsic parameters
  		float temp;
  		for(int i = 0; i < 3; i++)
  			for(int j = 0; j < 3; j++){
  				myfile >> temp;
  				//std::cout << temp << std::endl;
  				intrinsic.at<float>(i,j) = temp; 
  				//push_back(temp); 
  			}

  		//load distortion parameters
  		for(int i = 0; i < 5; i++){
  			myfile >> temp;
  			//std::cout << temp << std::endl;
  			distortion.at<float>(i) = temp;
  		}
		//fill up the 


		undistort(close, newcameramtx, intrinsic, distortion);

		absdiff(newcameramtx,close,diffImage);

		imwrite( "closeDiff.jpg", diffImage);

		undistort(far, newcameramtx, intrinsic, distortion);

		absdiff(newcameramtx,close,diffImage);

		imwrite( "farDiff.jpg", diffImage);

		undistort(turned, newcameramtx, intrinsic, distortion);

		absdiff(newcameramtx,close,diffImage);

		imwrite( "turnedDiff.jpg", diffImage);

		imshow("Roberts",diffImage);

		// std::cout << intrinsic << std::endl;
		// std::cout << distortion << std::endl;

	}
	//task 4
	else if(keyPress == '4'){

		Mat intrinsic = Mat::zeros(3,3,CV_32F);
		Mat distortion = Mat::zeros(1,5,CV_32F);
		std::vector<Point2f> imagePoints;
		std::vector<Point3f> objectPoints;

		//open file stream
		std::ifstream myfile;
  		myfile.open("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk2/task3/matricies.txt");

  		//load intrinsic parameters
  		float temp;
  		for(int i = 0; i < 3; i++)
  			for(int j = 0; j < 3; j++){
  				myfile >> temp;
  				//std::cout << temp << std::endl;
  				intrinsic.at<float>(i,j) = temp; 
  				//push_back(temp); 
  			}

  		//load distortion parameters
  		for(int i = 0; i < 5; i++){
  			myfile >> temp;
  			//std::cout << temp << std::endl;
  			distortion.at<float>(i) = temp;
  		}


  		myfile.close();
  		myfile.open("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk2/task3/DataPoints.txt");
  		
  		//load imagepoints parameters
  		float x,y,z;
  		for(int i = 0; i < 20; i++){
  			myfile >> x >> y;
  			// std::cout << x << "       " << y << std::endl;
  			imagePoints.push_back(Point2f(x, y));
  		}

  		//load objectpoints
  		for(int i = 0; i < 20; i++){
  			myfile >> x >> y >> z;
  			// std::cout << x << "       " << y << "       " << z << std::endl;
  			objectPoints.push_back(Point3f(x, y, z));
  		}

		
		Mat rvec;
		Mat tvec;
		solvePnP (objectPoints, imagePoints, intrinsic, distortion, rvec, tvec, false, SOLVEPNP_ITERATIVE);

		Rodrigues(rvec, rvec, noArray());
		std::cout << rvec << std::endl;
		std::cout << tvec << std::endl;


	}
	//task 5
	else if(keyPress == '5'){


	}
	//task6
	else if(keyPress == '6'){


	}
		

	// When everything done, release the video capture and write object
	//video.release();

	waitKey(0);
	// Closes all the windows
	destroyAllWindows();

	return 0;
}
