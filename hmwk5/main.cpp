#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;

double dist(Point2f pt1, Point2f pt2){
  return sqrt((pt2.x-pt1.x)*(pt2.x-pt1.x) + (pt2.y-pt1.y)*(pt2.y-pt1.y));
}

void reduce(vector<Point2f> prevPtsNonReduced, vector<Point2f> nextPtsNonReduced, vector<unsigned char> status, vector<float> error, vector<Point2f>& prevPts, vector<Point2f>& nextPts ){
  double maxDist = 10;
  double maxError = 20;
  for(int i = 0; i < prevPtsNonReduced.size(); i++){
    // if it meets conditions create a new vector
    // dist(prevPtsNonReduced[i],nextPtsNonReduced[i])< maxDist
    if( error[i] < maxError && status[i] == 1 ){
      prevPts.push_back(prevPtsNonReduced[i]);
      nextPts.push_back(nextPtsNonReduced[i]);
    }
  }
}

void takeVideo(VideoCapture cap, int pyrLevel, int skip, VideoWriter& writer){
  //variables
  Mat grayScale;

  //good features to track variables
  int maxCorners = 500;
  double qualityLevel = 0.1;
  double minDistance = 4;
  InputArray mask = noArray();
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  std::vector<Point2f> previousPoints;
  std::vector<Point2f> nextPoints;






  // double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count
  // cap.set(CV_CAP_PROP_POS_FRAMES,count-1); //Set index to last frame

  bool emptyFrame = false;
  Mat currentFrame;
  cap >> currentFrame;
  Mat previousFrame;
  cvtColor( currentFrame, grayScale, CV_BGR2GRAY );

  previousFrame = grayScale.clone();
  currentFrame = grayScale.clone();



  while(!emptyFrame)
  {
      //variables

      std::vector<unsigned char> status;
      std::vector<float> error;

      std::vector<Point2f> reducedPrev;
      std::vector<Point2f> reducedNext;

      for(int i = 0; i < (skip + 1); i++){
        cap >> currentFrame;
      }

      cvtColor( currentFrame, grayScale, CV_BGR2GRAY );
      emptyFrame = grayScale.empty();

      // bool success = cap.read(frame);
      if (emptyFrame){
        cout << "Cannot read  frame " << endl;
        // break;
      }
      else{



        //goodFeaturesToTrack(InputArray image, OutputArray corners,
        //int maxCorners, double qualityLevel, double minDistance,
        //InputArray mask=noArray(), int blockSize=3,
        //bool useHarrisDetector=false, double k=0.04 )


        goodFeaturesToTrack(previousFrame, previousPoints, maxCorners,
          qualityLevel, minDistance, mask, blockSize,useHarrisDetector,k);


        //calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg,
        //InputArray prevPts, InputOutputArray nextPts, OutputArray status,
        //OutputArray err, Size winSize=Size(21,21), int maxLevel=3,
        //TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
        //30, 0.01), int flags=0, double minEigThreshold=1e-4 )
        calcOpticalFlowPyrLK(previousFrame, grayScale, previousPoints, nextPoints, status,
          error, Size(21,21), pyrLevel,
          TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), 0.5, 0);



        //arrowedLine(Mat& img, Point pt1, Point pt2, const Scalar& color,
        //int thickness=1, int line_type=8, int shift=0, double tipLength=0.1)

        //take out non matches and simplify
        reduce(previousPoints, nextPoints, status, error, reducedPrev, reducedNext);

        cout << nextPoints.size() << " nextPoints size\n";
        cout << previousPoints.size() << " previousPoints size\n";
        cout << status.size() << " status size\n";

        cout << reducedNext.size() << " reduced nextPoints size\n";
        cout << reducedPrev.size() << " reduced prevPoints size\n";
        cout << reducedPrev.size()/(double)(previousPoints.size())<< " percentage\n\n\n";


        // cvtColor( currentFrame, currentFrame, CV_GRAY2BGR );
        for(int i = 0; i < reducedPrev.size(); i++){
          // arrowedLine(currentFrame, previousPoints[i], nextPoints[i], Scalar(255,0,0), 1, 8, 0, 0.1);
          circle(currentFrame, reducedPrev[i], 1, Scalar(0,255,0), 2, 8, 0);
          line(currentFrame, reducedPrev[i], reducedNext[i], Scalar(0,0,255), 2);
        }

        imshow("MyVideo", currentFrame);
        writer << currentFrame;
        previousFrame = grayScale.clone();
        // previousPoints = reducedNext;
      }



      // if(waitKey(0) == 27) break;
  }


}

int main( int argc, char** argv )
{
  //video screens
  namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);

  int task = 1;

  if(task == 1){


    VideoWriter VOut;

    VideoCapture cap1("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk5/video/MotionFieldVideo.mp4");
    if( !cap1.isOpened()){
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    Mat temp;
    cap1 >> temp;
    // Default resolution of the frame is obtained.The default resolution is system dependent.
  	int frame_width = temp.cols;
  	int frame_height = temp.rows;

  	// Initialize video write object (only done once). Change frame size to match your camera resolution.
  	VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(frame_width, frame_height), 1);
    //VOut.release();


    // input video, pyramid, skips, outVideo
    takeVideo(cap1, 0, 0, VOut);

    cap1.release();

    VideoCapture cap2("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk5/video/MotionFieldVideo.mp4");
    if( !cap2.isOpened()){
         cout << "Cannot open the video file" << endl;
         return -1;
    }



    takeVideo(cap2, 2, 20, VOut);
    cap2.release();
    VOut.release();
  }else if(task ==2){
    //templ_roi(point, size);
    //search template

  }







  return 0;
}
