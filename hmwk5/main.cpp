#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;


//globals




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

  std::vector<Mat> images;
  // std::vector<Point2f> nextPoints;

  // double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count
  // cap.set(CV_CAP_PROP_POS_FRAMES,count-1); //Set index to last frame

  bool emptyFrame = false;
  Mat currentFrame;

  for(int i = 0; i < (skip + 2); i++){
    cap >> currentFrame;
    images.push_back(currentFrame.clone());
  }

  Mat previousFrame;
  cvtColor( currentFrame, grayScale, CV_BGR2GRAY );
  cvtColor( images[0], previousFrame, CV_BGR2GRAY );

  currentFrame = grayScale.clone();
  while(!emptyFrame)
  {
      //variables

      std::vector<unsigned char> status;
      std::vector<float> error;

      std::vector<Point2f> reducedPrev;
      std::vector<Point2f> reducedNext;

      cap >> currentFrame;
      images.push_back(currentFrame.clone());
      images.erase(images.begin());

      cout << "vec size: " << images.size() << endl;
      cvtColor( images[0], previousFrame, CV_BGR2GRAY );

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

Point2f correctPoint(Point pointIn, int width, int height, int searchDimension){
  if((pointIn.x + searchDimension/2) >  width)
    pointIn.x = width - searchDimension;
  else if((pointIn.x - searchDimension/2) <  0)
    pointIn.x = 0;
  else
    pointIn.x = pointIn.x - searchDimension/2;

  if((pointIn.y + searchDimension/2) >  height)
    pointIn.y = height - searchDimension;
  else if((pointIn.y - searchDimension/2) <  0)
    pointIn.y = 0;
  else
    pointIn.y = pointIn.y - searchDimension/2;


  return pointIn;
}

Point2f MatchingMethod(Point2f searchLocation, Mat searchArea, Mat templ, Point2f pt)
{
  int match_method = cv::TM_SQDIFF_NORMED;
  Mat result;
  double threshold = 0.5;

  int result_cols =  searchArea.cols - templ.cols + 1;
  int result_rows = searchArea.rows - templ.rows + 1;
  result.create( result_rows, result_cols, CV_32FC1 );

  matchTemplate( searchArea, templ, result, match_method);
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
  double minVal, maxVal;
  Point minLoc, maxLoc;
  Point2f matchLoc;
  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
  // cout << "max value: " << maxVal << endl;

  // if (maxVal > threshold){
    // matchLoc.x = pt.x - result.cols/2.0 + minLoc.x;
    // matchLoc.y = pt.y - result.rows/2.0 + minLoc.y;
    matchLoc.x = searchLocation.x + templ.cols/2.0 + minLoc.x;
    matchLoc.y = searchLocation.y + templ.rows/2.0 + minLoc.y;
  // }else{
  //   matchLoc.x = -1;
  //   matchLoc.y = -1;
  // }

  return matchLoc;
}

void reduce2(vector<Point2f> prevPtsNonReduced, vector<Point2f> nextPtsNonReduced, vector<Point2f>& prevPts, vector<Point2f>& nextPts ){
  double maxDist = 25;
  // double maxError = 20;
  double maxThreshold = 10;

  for(int i = 0; i < prevPtsNonReduced.size(); i++){
    // if it meets conditions create a new vector
    if (dist(prevPtsNonReduced[i],nextPtsNonReduced[i])< maxDist){
      prevPts.push_back(prevPtsNonReduced[i]);
      nextPts.push_back(nextPtsNonReduced[i]);
    }
  }
}

void reduce3(vector<Point2f> prevPtsNonReduced, vector<Point2f> nextPtsNonReduced, vector<Point2f>& prevPts, vector<Point2f>& nextPts ){
  double maxDist = 25;
  // double maxError = 20;
  double maxThreshold = 10;
  Mat status;
  findFundamentalMat(prevPtsNonReduced, nextPtsNonReduced, FM_RANSAC, 3, 0.99,status);


  for(int i = 0; i < status.rows; i++){
    // if it meets conditions create a new vector
    if (status.at<uchar>(i,0)){
      prevPts.push_back(prevPtsNonReduced[i]);
      nextPts.push_back(nextPtsNonReduced[i]);
    }
  }
}

void takeVideoT2(VideoCapture cap, int pyrLevel, int skip, VideoWriter& writer){
  //variables
  Mat grayScale;

  //good features to track variables
  int maxCorners = 500;
  double qualityLevel = 0.01;
  double minDistance = 25;
  InputArray mask = noArray();
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  std::vector<Point2f> previousPoints;
  std::vector<Mat> images;

  bool emptyFrame = false;
  Mat currentFrame;

  for(int i = 0; i < (skip + 2); i++){
    cap >> currentFrame;
    images.push_back(currentFrame.clone());
  }

  Mat previousFrame;
  cvtColor( currentFrame, grayScale, CV_BGR2GRAY );
  cvtColor( images[0], previousFrame, CV_BGR2GRAY );

  currentFrame = grayScale.clone();



  while(!emptyFrame)
  {
      //variables

      std::vector<unsigned char> status;
      std::vector<float> error;

      std::vector<Point2f> reducedPrev;
      std::vector<Point2f> reducedNext;
      vector<Rect> searchVec;
      vector<Rect> templateVec;

      std::vector<Point2f> nextPoints;


      cap >> currentFrame;
      images.push_back(currentFrame.clone());
      images.erase(images.begin());

      cout << "vec size: " << images.size() << endl;
      cvtColor( images[0], previousFrame, CV_BGR2GRAY );

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


        // //calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg,
        // //InputArray prevPts, InputOutputArray nextPts, OutputArray status,
        // //OutputArray err, Size winSize=Size(21,21), int maxLevel=3,
        // //TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
        // //30, 0.01), int flags=0, double minEigThreshold=1e-4 )
        // calcOpticalFlowPyrLK(previousFrame, grayScale, previousPoints, nextPoints, status,
        //   error, Size(21,21), pyrLevel,
        //   TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), 0.5, 0);
        //
        //
        //
        // //arrowedLine(Mat& img, Point pt1, Point pt2, const Scalar& color,
        // //int thickness=1, int line_type=8, int shift=0, double tipLength=0.1)
        //
        // //take out non matches and simplify

        int tmpW = 10;
        int srcW = tmpW * 10;
        Size tmpSize = Size(tmpW,tmpW);
        Size srcSize = Size(srcW, srcW);

        for(int i = 0; i < previousPoints.size(); i++)
        {
          // void correctPoint(Point& pointIn, int width, int height, int searchDimension){
          Point2f tmpCorrected = correctPoint(previousPoints[i], previousFrame.cols, previousFrame.rows, tmpW);
          Point2f srcCorrected = correctPoint(previousPoints[i], previousFrame.cols, previousFrame.rows, srcW);

          Rect templ_roi(tmpCorrected, tmpSize);
          Mat templ = previousFrame(templ_roi);
          Rect search_roi(srcCorrected, srcSize);
          Mat searchImage = grayScale(search_roi);

          // Point2f MatchingMethod(Mat searchArea, Mat templ)
          // imshow("searchImage",searchImage);
          // imshow("templateImage",templ);
          Point2f temp = MatchingMethod(srcCorrected, searchImage, templ, tmpCorrected);
          // nextPoints.push_back(MatchingMethod(searchImage, templ, tmpCorrected));
          if(!(temp.x == -1)){
            nextPoints.push_back(temp);
            searchVec.push_back(search_roi);
            templateVec.push_back(templ_roi);
          }
        }


        reduce2(previousPoints, nextPoints, reducedPrev, reducedNext);

        cout << nextPoints.size() << " nextPoints size\n";
        cout << previousPoints.size() << " previousPoints size\n";
        // cout << status.size() << " status size\n";

        cout << reducedNext.size() << " reduced nextPoints size\n";
        cout << reducedPrev.size() << " reduced prevPoints size\n";
        cout << reducedPrev.size()/(double)(previousPoints.size())<< " percentage\n\n\n";

        // cvtColor( currentFrame, currentFrame, CV_GRAY2BGR );
        for(int i = 0; i < reducedPrev.size(); i++){
          // arrowedLine(currentFrame, previousPoints[i], nextPoints[i], Scalar(255,0,0), 1, 8, 0, 0.1);
          circle(currentFrame, reducedPrev[i], 1, Scalar(0,255,0), 2, 8, 0);
          line(currentFrame, reducedPrev[i], reducedNext[i], Scalar(0,0,255), 2);
          // rectangle( currentFrame, templateVec[i], Scalar( 255, 0, 0 ), 2, 1 );
          // rectangle( currentFrame, searchVec[i], Scalar( 0, 255, 0 ), 2, 1 );
        }

        imshow("MyVideo", currentFrame);
        writer << currentFrame;

        // waitKey(0);
        // previousPoints = reducedNext;
      }



      // if(waitKey(0) == 27) break;
  }


}

void takeVideoT3(VideoCapture cap, int pyrLevel, int skip, VideoWriter& writer){
  //variables
  Mat grayScale;

  //good features to track variables
  int maxCorners = 500;
  double qualityLevel = 0.01;
  double minDistance = 25;
  InputArray mask = noArray();
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  std::vector<Point2f> previousPoints;
  std::vector<Mat> images;

  bool emptyFrame = false;
  Mat currentFrame;

  for(int i = 0; i < (skip + 2); i++){
    cap >> currentFrame;
    images.push_back(currentFrame.clone());
  }

  Mat previousFrame;
  cvtColor( currentFrame, grayScale, CV_BGR2GRAY );
  cvtColor( images[0], previousFrame, CV_BGR2GRAY );

  currentFrame = grayScale.clone();



  while(!emptyFrame)
  {
      //variables

      std::vector<unsigned char> status;
      std::vector<float> error;

      std::vector<Point2f> reducedPrev;
      std::vector<Point2f> reducedNext;
      vector<Rect> searchVec;
      vector<Rect> templateVec;

      std::vector<Point2f> nextPoints;


      cap >> currentFrame;
      images.push_back(currentFrame.clone());
      images.erase(images.begin());

      cout << "vec size: " << images.size() << endl;
      cvtColor( images[0], previousFrame, CV_BGR2GRAY );

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


        // //calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg,
        // //InputArray prevPts, InputOutputArray nextPts, OutputArray status,
        // //OutputArray err, Size winSize=Size(21,21), int maxLevel=3,
        // //TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
        // //30, 0.01), int flags=0, double minEigThreshold=1e-4 )
        // calcOpticalFlowPyrLK(previousFrame, grayScale, previousPoints, nextPoints, status,
        //   error, Size(21,21), pyrLevel,
        //   TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), 0.5, 0);
        //
        //
        //
        // //arrowedLine(Mat& img, Point pt1, Point pt2, const Scalar& color,
        // //int thickness=1, int line_type=8, int shift=0, double tipLength=0.1)
        //
        // //take out non matches and simplify

        int tmpW = 10;
        int srcW = tmpW * 10;
        Size tmpSize = Size(tmpW,tmpW);
        Size srcSize = Size(srcW, srcW);

        for(int i = 0; i < previousPoints.size(); i++)
        {
          // void correctPoint(Point& pointIn, int width, int height, int searchDimension){
          Point2f tmpCorrected = correctPoint(previousPoints[i], previousFrame.cols, previousFrame.rows, tmpW);
          Point2f srcCorrected = correctPoint(previousPoints[i], previousFrame.cols, previousFrame.rows, srcW);

          Rect templ_roi(tmpCorrected, tmpSize);
          Mat templ = previousFrame(templ_roi);
          Rect search_roi(srcCorrected, srcSize);
          Mat searchImage = grayScale(search_roi);

          // Point2f MatchingMethod(Mat searchArea, Mat templ)
          // imshow("searchImage",searchImage);
          // imshow("templateImage",templ);
          Point2f temp = MatchingMethod(srcCorrected, searchImage, templ, tmpCorrected);
          // nextPoints.push_back(MatchingMethod(searchImage, templ, tmpCorrected));
          if(!(temp.x == -1)){
            nextPoints.push_back(temp);
            searchVec.push_back(search_roi);
            templateVec.push_back(templ_roi);
          }
        }


        reduce3(previousPoints, nextPoints, reducedPrev, reducedNext);

        cout << nextPoints.size() << " nextPoints size\n";
        cout << previousPoints.size() << " previousPoints size\n";
        // cout << status.size() << " status size\n";

        cout << reducedNext.size() << " reduced nextPoints size\n";
        cout << reducedPrev.size() << " reduced prevPoints size\n";
        cout << reducedPrev.size()/(double)(previousPoints.size())<< " percentage\n\n\n";

        // cvtColor( currentFrame, currentFrame, CV_GRAY2BGR );
        for(int i = 0; i < reducedPrev.size(); i++){
          // arrowedLine(currentFrame, previousPoints[i], nextPoints[i], Scalar(255,0,0), 1, 8, 0, 0.1);
          circle(currentFrame, reducedPrev[i], 1, Scalar(0,255,0), 2, 8, 0);
          line(currentFrame, reducedPrev[i], reducedNext[i], Scalar(0,0,255), 2);
          // rectangle( currentFrame, templateVec[i], Scalar( 255, 0, 0 ), 2, 1 );
          // rectangle( currentFrame, searchVec[i], Scalar( 0, 255, 0 ), 2, 1 );
        }

        imshow("MyVideo", currentFrame);
        writer << currentFrame;

        // waitKey(0);
        // previousPoints = reducedNext;
      }



      // if(waitKey(0) == 27) break;
  }


}

int main( int argc, char** argv )
{
  int frame_width;
  int frame_height;
  //video screens
  namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);
  // namedWindow( "Source Image", WINDOW_AUTOSIZE );
  // namedWindow( "Result window", WINDOW_AUTOSIZE );

  int task = 3;

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
  	frame_width = temp.cols;
  	frame_height = temp.rows;

  	// Initialize video write object (only done once). Change frame size to match your camera resolution.
  	VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(frame_width, frame_height), 1);
    //VOut.release();


    // input video, pyramid, skips, outVideo
    takeVideo(cap1, 2, 0, VOut);

    cap1.release();

    VideoCapture cap2("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk5/video/MotionFieldVideo.mp4");
    if( !cap2.isOpened()){
         cout << "Cannot open the video file" << endl;
         return -1;
    }



    takeVideo(cap2, 2, 10, VOut);
    cap2.release();
    VOut.release();


  }else if(task ==2){

    VideoWriter VOut;

    VideoCapture cap1("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk5/video/MotionFieldVideo.mp4");
    if( !cap1.isOpened()){
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    Mat temp;
    cap1 >> temp;
    // Default resolution of the frame is obtained.The default resolution is system dependent.
  	frame_width = temp.cols;
  	frame_height = temp.rows;

  	// Initialize video write object (only done once). Change frame size to match your camera resolution.
  	VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(frame_width, frame_height), 1);
    //VOut.release();


    // input video, pyramid, skips, outVideo
    takeVideoT2(cap1, 0, 0, VOut);

    cap1.release();

    VideoCapture cap2("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk5/video/MotionFieldVideo.mp4");
    if( !cap2.isOpened()){
         cout << "Cannot open the video file" << endl;
         return -1;
    }



    takeVideoT2(cap2, 0, 10, VOut);
    cap2.release();
    VOut.release();
    //templ_roi(point, size);
    //search template







  }

  else if(task == 3){
    VideoWriter VOut;

    VideoCapture cap1("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk5/video/MotionFieldVideo.mp4");
    if( !cap1.isOpened()){
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    Mat temp;
    cap1 >> temp;
    // Default resolution of the frame is obtained.The default resolution is system dependent.
    frame_width = temp.cols;
    frame_height = temp.rows;

    // Initialize video write object (only done once). Change frame size to match your camera resolution.
    VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(frame_width, frame_height), 1);
    //VOut.release();


    // input video, pyramid, skips, outVideo
    takeVideoT3(cap1, 0, 0, VOut);

    cap1.release();


    VOut.release();
    //templ_roi(point, size);
    //search template



  }







  return 0;
}
