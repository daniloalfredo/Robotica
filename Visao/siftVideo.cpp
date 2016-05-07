/**
 * @file Sift_FlannMatcher
 * @brief Sift detector + descriptor + FLANN Matcher
 * @author A. Huaman
 */

#include "opencv2/opencv_modules.hpp"
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++.h>
# include "opencv2/core/core.hpp"
# include "opencv2/features2d/features2d.hpp"
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/nonfree/features2d.hpp"
 #include <time.h>



using namespace cv;
using namespace std;

/**
 * @function main
 * @brief Main function
 */

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    cout << tstruct.tm_year << endl;

    strftime(buf, sizeof(buf), "%Y-%m-%d.avi", &tstruct);
    return buf;
}

int main( int argc, char** argv )
{

  Mat img_1 = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
  Mat img_10 = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );

  VideoCapture cap(0);

  if(!cap.isOpened())  // check if we succeeded
        return -1;

  cv::namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); 

  if( !img_1.data || !img_10.data)
  { printf(" --(!) Error reading images \n"); return -1; }

  //-- Step 1: Detect the keypoints using Sift Detector
  int minHessian = 400;

  SiftFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_1;
  std::vector<KeyPoint> keypoints_10;

  detector.detect( img_1, keypoints_1 );
  detector.detect( img_10, keypoints_10 );
  
  

  //-- Step 2: Calculate descriptors (feature vectors)
  SiftDescriptorExtractor extractor;

  Mat descriptors_1,descriptors_10;

  extractor.compute( img_1, keypoints_1, descriptors_1 );
  extractor.compute( img_10, keypoints_10, descriptors_10 );
  

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

   std::cout << "Frame Size = " << dWidth << "x" << dHeight << std::endl;

  cv::Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
 cout << "currenttime: " <<currentDateTime() << endl;
 cv::VideoWriter oVideoWriter (currentDateTime(), CV_FOURCC('P','I','M','1'), 20, frameSize, true); //initialize the VideoWriter object 

   if ( !oVideoWriter.isOpened() ) 
   {
        std::cout << "ERROR: Failed to write the video" << std::endl;
        return -1;
   }




  while(1)
  {
        Mat img_2,frame;

        bool bSuccess = cap.read(img_2); // read a new frame from video

        if (!bSuccess) //if not success, break loop
       {
             std::cout << "ERROR: Cannot read a frame from video file" << std::endl;
             break;
        }


        frame = img_2;
        cvtColor(img_2, img_2, CV_BGR2GRAY);
        
         std::vector<KeyPoint> keypoints_2;
  Mat descriptors_2;
  detector.detect( img_2, keypoints_2 );
  extractor.compute( img_2, keypoints_2, descriptors_2 );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

  FlannBasedMatcher matcher2;
  std::vector< DMatch > matches2;
  matcher2.match( descriptors_10, descriptors_2, matches2 );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

    //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_2.rows; i++ )
  { double dist = matches2[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  //printf("-- Max dist : %f \n", max_dist );
  //printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;
  std::vector< DMatch > good_matches2;

  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance <= max(2*min_dist, 0.02) )
    { good_matches.push_back( matches[i]); }
  }

    for( int i = 0; i < descriptors_2.rows; i++ )
  { if( matches2[i].distance <= max(2*min_dist, 0.02) )
    { good_matches2.push_back( matches2[i]); }
  }

  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Show detected matches
  imshow( "Good Matches", img_matches );

  //for( int i = 0; i < (int)good_matches.size(); i++ )
  //{ printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

//  

    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;  
    cv::Point textOrg(10, 130);

  if(good_matches.size() >= 20 || good_matches2.size() >= 20)
  {
    string text = "Achou";
    putText(frame, text, textOrg, fontFace, fontScale,Scalar(0,0,255), thickness, 8);
    cout  << "Achou :"<<good_matches.size() << " " << good_matches2.size() << endl;
  }
  else
  {
    string text = "Nao Achou";
    putText(frame, text, textOrg, fontFace, fontScale,Scalar(0,0,255), thickness, 8);
    cout  << "Nao Achou :"<<good_matches.size()  << " " << good_matches2.size()<< endl;
  }
  //cout  << good_matches.size() << endl;


        if(waitKey(30) >= 0) break;
  oVideoWriter.write(frame); //writer the frame into the file
  cv::imshow("MyVideo", frame);
 
  }



  return 0;
}

/**
 * @function readme
 */
/*
void readme()
{ printf(" Usage: ./Sift_FlannMatcher <img1> <img2>\n"); }
*/