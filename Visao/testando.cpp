/**
 * @file Surf_FlannMatcher
 * @brief Surf detector + descriptor + FLANN Matcher
 * @author A. Huaman
 */
 
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <bits/stdc++.h> 
 
 
using namespace cv;
using namespace std;
 
/**
 * @function main
 * @brief Main function
 */
 

int main( int argc, char** argv )
{ 
  VideoCapture cap(0);
 
  if(!cap.isOpened())  // check if we succeeded
        return -1;
 
  cv::namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);
 
  //-- Step 1: Detect the keypoints using Surf Detector 
  int minHessian = 400;
  SurfFeatureDetector detector( minHessian );
 
  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;
  std::vector<KeyPoint> keypoints_1;
  Mat descriptors_1;

  FileStorage fs("dictionary.yml", FileStorage::READ);
  fs["vocabulary"] >> descriptors_1;
  fs.release();    
 
  while(1)
  {
    Mat img_2,frame;
 
    bool bSuccess = cap.read(img_2); // read a new frame from video
 
    if (!bSuccess) //if not success, break loop
    {
      std::cout << "ERROR: Cannot read a frame from camera file" << std::endl;
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
   
    double max_dist = 0; double min_dist = 100;
   
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }
 
  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;
 
  for( int i = 0; i < descriptors_1.rows; i++ )
  { 
    if( matches[i].distance <= max(2*min_dist, 0.02) )
    { 
      good_matches.push_back( matches[i]); 
    }
  }
 

  int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
  double fontScale = 2;
  int thickness = 3;  
  cv::Point textOrg(10, 130);
 
  if(good_matches.size() >= 35)
  {
    string text = "Achou";
    putText(frame, text, textOrg, fontFace, fontScale,Scalar(0,0,255), thickness, 8);
    cout  << "Achou :"<<good_matches.size() << endl;
    Mat teste = frame;

    Mat aux = imread( "obj1_01.jpg", CV_LOAD_IMAGE_GRAYSCALE );

    if(!aux.data)
      {cout << "NULL" << endl;}

     
    cv::SurfFeatureDetector detector( minHessian );

    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

    detector.detect( aux, keypoints_object );
    detector.detect( teste, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;

    cv::Mat descriptors_object, descriptors_scene;

    extractor.compute( aux, keypoints_object, descriptors_object );
    extractor.compute( teste, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }

    //  printf("-- Max dist : %f \n", max_dist );
    //  printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 3*min_dist )
       { good_matches.push_back( matches[i]); }
    }

    cv::Mat frame;
    cv::drawMatches( aux, keypoints_object, teste, keypoints_scene,
                 good_matches, frame, cv::Scalar::all(-1), cv::Scalar::all(-1),
                 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
      //-- Get the keypoints from the good matches
      obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
      scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    cv::Mat H = findHomography( obj, scene, CV_RANSAC );

     //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( aux.cols, 0 );
    obj_corners[2] = cvPoint( aux.cols, aux.rows ); obj_corners[3] = cvPoint( 0, aux.rows );
    std::vector<cv::Point2f> scene_corners(4);

    cv::perspectiveTransform( obj_corners, scene_corners, H);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    cv::line( frame, scene_corners[0] + cv::Point2f( aux.cols, 0), scene_corners[1] + cv::Point2f( aux.cols, 0), cv::Scalar(0, 255, 0), 4 );
    cv::line( frame, scene_corners[1] + cv::Point2f( aux.cols, 0), scene_corners[2] + cv::Point2f( aux.cols, 0), cv::Scalar( 0, 255, 0), 4 );
    cv::line( frame, scene_corners[2] + cv::Point2f( aux.cols, 0), scene_corners[3] + cv::Point2f( aux.cols, 0), cv::Scalar( 0, 255, 0), 4 );
    cv::line( frame, scene_corners[3] + cv::Point2f( aux.cols, 0), scene_corners[0] + cv::Point2f( aux.cols, 0), cv::Scalar( 0, 255, 0), 4 );

      //-- Show detected matches
    cv::imshow( "Good Matches & Object detection", frame );

  }
  else
  {
    string text = "Nao Achou";
    putText(frame, text, textOrg, fontFace, fontScale,Scalar(0,0,255), thickness, 8);
    cout  << "Nao Achou :"<<good_matches.size() << endl;
  }
    //cout  << good_matches.size() << endl;
   
   
    if(waitKey(30) >= 0) break;
    cv::imshow("MyVideo", frame);
   
  }
 
 
 
  return 0;
}