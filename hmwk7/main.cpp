#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;


double dist(Point2f pt1, Point2f pt2){
  return sqrt((pt2.x-pt1.x)*(pt2.x-pt1.x) + (pt2.y-pt1.y)*(pt2.y-pt1.y));
}

double dist2(Point2f pt1, Point2f pt2){
  return (pt2.x-pt1.x);
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

void takeVideo(int pyrLevel, ofstream& writer){
  //variables
  Mat grayScale;

  //good features to track variables
  int maxCorners = 1;
  double qualityLevel = .0001;
  double minDistance = 4;
  InputArray mask = noArray();
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  std::vector<Point2f> previousPoints;

  std::vector<Point2f> nextPoints;
  vector<double> previousDistances;
  double previousDistance;




  std::vector<Mat> images;
  // std::vector<Point2f> nextPoints;

  // double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count
  // cap.set(CV_CAP_PROP_POS_FRAMES,count-1); //Set index to last frame

  bool emptyFrame = false;
  Mat currentFrame;
  Mat previousFrame;

  stringstream ss;

  ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk7/images/T" << 1 << ".jpg";
  previousFrame = imread(ss.str());
  ss.str("");


  // cvtColor( currentFrame, grayScale, CV_BGR2GRAY );
  cvtColor( previousFrame, previousFrame, CV_BGR2GRAY );

  // get the can from the roi
  Rect2d initialRoi = selectROI(previousFrame);

  //points to track
  float trackLines = 6;
  float marginSpacing = 20;
  float xSpacing = 15;


  float height = initialRoi.height;
  float width = initialRoi.width;

  float iterateSpacing = (height - marginSpacing)/trackLines;
  float searchDimension = iterateSpacing;

  // shift all the points by the roi position
  for(int i = 0; i < trackLines; i++){
    vector<Point2f> tempPoints;
    vector<Point2f> tempPointsR;

    float depth = initialRoi.y + i * iterateSpacing + marginSpacing/2;
    // Select ROI
    Rect2d roiL = Rect2d(initialRoi.x + xSpacing, depth , searchDimension, searchDimension);

    // Crop image
    Mat croppedL = previousFrame(roiL);

    imshow("MyVideo", croppedL);
    waitKey(0);

    // currentFrame = grayScale.clone();

    goodFeaturesToTrack(croppedL, tempPoints, maxCorners,
    qualityLevel, minDistance, mask, blockSize,useHarrisDetector,k);

    cout << tempPoints[1].x << " tempPointsL before x\n";
    cout << tempPoints[1].y << " tempPointsL before y\n";

    cout << roiL.x << " roiL x\n";
    cout << roiL.y << " roiL y\n";


    tempPoints[1].x += roiL.x;
    tempPoints[1].y += roiL.y;

    cout << tempPoints[1].x << " tempPointsL after x\n";
    cout << tempPoints[1].y << " tempPointsL after y\n";

    rectangle(previousFrame,roiL,Scalar(255,0,0),1,8,0);


    previousPoints.push_back(tempPoints[1]);
    // tempPoints.clear();

    Rect2d roiR = Rect2d(initialRoi.x + width - xSpacing - searchDimension, depth , searchDimension, searchDimension);

    // Crop image
    Mat croppedR = previousFrame(roiR);

    // currentFrame = grayScale.clone();
    imshow("MyVideo", croppedR);
    waitKey(0);

    goodFeaturesToTrack(croppedR, tempPointsR, maxCorners,
    qualityLevel, minDistance, mask, blockSize,useHarrisDetector,k);

    cout << tempPointsR[1].x << " tempPointsR before x\n";
    cout << tempPointsR[1].y << " tempPointsR before y\n";

    cout << roiR.x << " roiR x\n";
    cout << roiR.y << " roiR y\n";

    tempPointsR[1].x += roiR.x;
    tempPointsR[1].y += roiR.y;

    cout << tempPointsR[1].x << " tempPointsR after x\n";
    cout << tempPointsR[1].y << " tempPointsR after y\n";

    rectangle(previousFrame,roiR,Scalar(255,0,0),1,8,0);



    imshow("MyVideo", previousFrame);

    waitKey(0);

    previousPoints.push_back(tempPointsR[1]);
    // tempPoints.clear();


  }

  bool firstTime = true;

  for(int i = 2; i <= 18; i++)
  {
      //variables
      // previousDistances.clear();
      std::vector<unsigned char> status;
      std::vector<float> error;

      std::vector<Point2f> reducedPrev;
      std::vector<Point2f> reducedNext;

      vector<double> currentDistances;
      double currentDistance;

      vector<double> tau;


      // vector<double> A;


      ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk7/images/T" << i << ".jpg";
      currentFrame = imread(ss.str());
      ss.str("");
      // images.push_back(currentFrame.clone());
      // images.erase(images.begin());

      // cout << "vec size: " << images.size() << endl;
      // cvtColor( images[0], previousFrame, CV_BGR2GRAY );

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
        double sum = 0;
        double temp = 0;
        double temp2 = 0;
        double diff = 0;
        double A = 0;
        double tau;
        double count = 0;

        // draw lines
        for(int i = 0; i < previousPoints.size(); i++){
          // arrowedLine(currentFrame, previousPoints[i], nextPoints[i], Scalar(255,0,0), 1, 8, 0, 0.1);
          cout << previousPoints[i] << " previous point\n";
          cout << nextPoints[i] << " next point\n";
          circle(currentFrame, previousPoints[i], 1, Scalar(0,255,0), 10, 8, 0);
          line(currentFrame, previousPoints[i], nextPoints[i], Scalar(0,0,255), 2);
        }


        for(int i = 0; i < previousPoints.size(); i = i + 2){
          // sum += dist(reducedPrev[i],reducedNext[i]);
          temp = abs(dist2(previousPoints[i],previousPoints[i+1]));
          temp2 = abs(dist2(nextPoints[i],nextPoints[i+1]));
          // cout << temp << " " << temp2 << endl;

          if(temp != 0){
            A = temp2/temp;
          }else{
            A = -5;
          }

          if( A > 1){
            sum += A / (A - 1.0);
            count++;
          }
          // if(!firstTime && temp < previousDistances[i]){
          //   A = previousDistances[i]/temp;
          //   sum += A / (A - 1.0);
          //   count++;
          // }
        }

        // currentDistance = sum/reducedPrev.size();
        writer << sum/count << ",";
        // cout << previousDistance << " previous distance\n";
        // cout << currentDistance << " current distance\n";



        imshow("MyVideo", currentFrame);
        waitKey(0);

        // if(!firstTime && currentDistance > previousDistance){
        //   A = currentDistance/previousDistance;
        //   writer << (A / ( A - 1.0)) << ",";
          // cout << previousDistance << " previous distance\n";
          // cout << currentDistance << " current distance\n";
        // }


        // writer << A << ",";
        previousFrame = grayScale.clone();
        previousPoints = nextPoints;
        previousDistances = currentDistances;
        firstTime = false;
      }



      // if(waitKey(0) == 27) break;
  }


}



void takeVideo3(int pyrLevel, ofstream& writer, float f){

  float diameter = 59; //mm

  //variables
  Mat grayScale;

  //good features to track variables
  int maxCorners = 1;
  double qualityLevel = .0001;
  double minDistance = 4;
  InputArray mask = noArray();
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  std::vector<Point2f> previousPoints;

  std::vector<Point2f> nextPoints;
  vector<double> previousDistances;
  double previousDistance;




  std::vector<Mat> images;
  // std::vector<Point2f> nextPoints;

  // double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count
  // cap.set(CV_CAP_PROP_POS_FRAMES,count-1); //Set index to last frame

  bool emptyFrame = false;
  Mat currentFrame;
  Mat previousFrame;

  stringstream ss;

  ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk7/images/T" << 1 << ".jpg";
  previousFrame = imread(ss.str());
  ss.str("");


  // cvtColor( currentFrame, grayScale, CV_BGR2GRAY );
  cvtColor( previousFrame, previousFrame, CV_BGR2GRAY );

  // get the can from the roi
  Rect2d initialRoi = selectROI(previousFrame);

  //points to track
  float trackLines = 1;
  float searchDimension = 20;

  float height = initialRoi.height;
  float width = initialRoi.width;
  float cornerLX = initialRoi.x - searchDimension/2;
  float cornerLY = initialRoi.y - searchDimension/2;
  float cornerRX = initialRoi.x + width - searchDimension/2;
  float cornerRY = initialRoi.y - searchDimension/2;


  // shift all the points by the roi position
  for(int i = 0; i < trackLines; i++){
    vector<Point2f> tempPoints;
    vector<Point2f> tempPointsR;

    // Select ROI
    Rect2d roiL = Rect2d(cornerLX, cornerLY, searchDimension, searchDimension);

    // Crop image
    Mat croppedL = previousFrame(roiL);

    imshow("MyVideo", croppedL);
    waitKey(0);



    // currentFrame = grayScale.clone();

    goodFeaturesToTrack(croppedL, tempPoints, maxCorners,
    qualityLevel, minDistance, mask, blockSize,useHarrisDetector,k);

    cout << tempPoints[0].x << " tempPointsL before x\n";
    cout << tempPoints[0].y << " tempPointsL before y\n";

    cout << roiL.x << " roiL x\n";
    cout << roiL.y << " roiL y\n";


    tempPoints[0].x += roiL.x;
    tempPoints[0].y += roiL.y;

    cout << tempPoints[0].x << " tempPointsL after x\n";
    cout << tempPoints[0].y << " tempPointsL after y\n";

    rectangle(previousFrame,roiL,Scalar(255,0,0),1,8,0);


    previousPoints.push_back(tempPoints[0]);
    // tempPoints.clear();



    Rect2d roiR = Rect2d(cornerRX, cornerRY, searchDimension, searchDimension);

    // Crop image
    Mat croppedR = previousFrame(roiR);

    // currentFrame = grayScale.clone();
    imshow("MyVideo", croppedR);
    waitKey(0);



    goodFeaturesToTrack(croppedR, tempPointsR, maxCorners,
    qualityLevel, minDistance, mask, blockSize,useHarrisDetector,k);

    cout << tempPointsR[0].x << " tempPointsR before x\n";
    cout << tempPointsR[0].y << " tempPointsR before y\n";

    cout << roiR.x << " roiR x\n";
    cout << roiR.y << " roiR y\n";

    tempPointsR[0].x += roiR.x;
    tempPointsR[0].y += roiR.y;

    cout << tempPointsR[0].x << " tempPointsR after x\n";
    cout << tempPointsR[0].y << " tempPointsR after y\n";

    rectangle(previousFrame,roiR,Scalar(255,0,0),1,8,0);



    imshow("MyVideo", previousFrame);

    waitKey(0);

    previousPoints.push_back(tempPointsR[0]);
    // tempPoints.clear();


  }



  bool firstTime = true;

  for(int i = 2; i <= 18; i++)
  {
      //variables
      // previousDistances.clear();
      std::vector<unsigned char> status;
      std::vector<float> error;

      std::vector<Point2f> reducedPrev;
      std::vector<Point2f> reducedNext;

      vector<double> currentDistances;
      double currentDistance;

      vector<double> tau;


      // vector<double> A;


      ss << "/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk7/images/T" << i << ".jpg";
      currentFrame = imread(ss.str());
      ss.str("");
      // images.push_back(currentFrame.clone());
      // images.erase(images.begin());

      // cout << "vec size: " << images.size() << endl;
      // cvtColor( images[0], previousFrame, CV_BGR2GRAY );

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
        double sum = 0;
        double temp = 0;
        double temp2 = 0;
        double diff = 0;
        double A = 0;
        double tau;
        double count = 0;

        // draw lines
        for(int i = 0; i < previousPoints.size(); i++){
          // arrowedLine(currentFrame, previousPoints[i], nextPoints[i], Scalar(255,0,0), 1, 8, 0, 0.1);
          cout << previousPoints[i] << " previous point\n";
          cout << nextPoints[i] << " next point\n";
          circle(currentFrame, previousPoints[i], 1, Scalar(0,255,0), 10, 8, 0);
          line(currentFrame, previousPoints[i], nextPoints[i], Scalar(0,0,255), 2);
        }


        // for(int i = 0; i < previousPoints.size(); i = i + 2){
        //   // sum += dist(reducedPrev[i],reducedNext[i]);
        //   temp = abs(dist2(previousPoints[i],previousPoints[i+1]));
        //   temp2 = abs(dist2(nextPoints[i],nextPoints[i+1]));
        //   // cout << temp << " " << temp2 << endl;
        //
        //   if(temp != 0){
        //     A = temp2/temp;
        //   }else{
        //     A = -5;
        //   }
        //
        //   if( A > 1){
        //     sum += A / (A - 1.0);
        //     count++;
        //   }
        //   // if(!firstTime && temp < previousDistances[i]){
        //   //   A = previousDistances[i]/temp;
        //   //   sum += A / (A - 1.0);
        //   //   count++;
        //   // }
        // }
        //
        // // currentDistance = sum/reducedPrev.size();
        // writer << sum/count << ",";
        double difference = previousPoints[1].x - previousPoints[0].x;
        writer << f * diameter / difference << ", ";
        // cout << previousDistance << " previous distance\n";
        // cout << currentDistance << " current distance\n";



        imshow("MyVideo", currentFrame);
        waitKey(0);

        // if(!firstTime && currentDistance > previousDistance){
        //   A = currentDistance/previousDistance;
        //   writer << (A / ( A - 1.0)) << ",";
          // cout << previousDistance << " previous distance\n";
          // cout << currentDistance << " current distance\n";
        // }


        // writer << A << ",";
        previousFrame = grayScale.clone();
        previousPoints = nextPoints;
        previousDistances = currentDistances;
        firstTime = false;

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

  //task 1 and 2
  if(task == 1){


    VideoWriter VOut;

    // VideoCapture cap1("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk5/video/MotionFieldVideo.mp4");
    // if( !cap1.isOpened()){
    //      cout << "Cannot open the video file" << endl;
    //      return -1;
    // }

    // Mat temp;
    // cap1 >> temp;
    // // Default resolution of the frame is obtained.The default resolution is system dependent.
  	// frame_width = temp.cols;
  	// frame_height = temp.rows;

  	// Initialize video write object (only done once). Change frame size to match your camera resolution.
  	// VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G') , 30, Size(frame_width, frame_height), 1);
    //VOut.release();


    // input video, pyramid, skips, outVideo
    ofstream myfile;
    myfile.open ("task1.txt");


    takeVideo(2, myfile);
    myfile.close();

    // cap1.release();




  }else if(task == 3){

    Mat cameraMat;
    Mat distortion;
    FileStorage fin("/home/justin/Documents/school/roboticVision/JustinsRepo/hmwk7/params.yaml", cv::FileStorage::READ);
    fin["mtx"] >> cameraMat;
    fin["dist"] >> distortion;

    float f = (float)cameraMat.at<double>(0,0);

    cout << f << " f\n";

    fin.release();

    cout  << " \ncameraMat---\n" << cameraMat <<  "\ndistortion---\n" << distortion << endl;

    VideoWriter VOut;

    ofstream myfile;
    myfile.open ("task1.txt");


    takeVideo3(0, myfile, f);
    myfile.close();
  }


return 0;

}
