#ifndef HOMOGRAPHY
#define HOMOGRAPHY

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

typedef struct upperDownPoints
{
  cv::Point up;
  cv::Point down;
}UpperDownPoints;


UpperDownPoints findPointsOfCamp(std::string side,std::string imageName,Mat img_scene)
{
  UpperDownPoints returnPoints;

  cv::Mat img_object = cv::imread( imageName, CV_LOAD_IMAGE_GRAYSCALE );
  cv::cvtColor(img_scene, img_scene, CV_BGR2GRAY);
  imshow("TSKAKOSA",img_scene);
  /*cv::Mat*/ //img_scene = cv::imread("images/camp.png", CV_LOAD_IMAGE_GRAYSCALE );

  if( !img_object.data || !img_scene.data )
  { std::cout<< " --(!) Error reading images " << std::endl; return returnPoints; }

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  cv::SurfFeatureDetector detector( minHessian );

  std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( img_object, keypoints_object );
  detector.detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  cv::SurfDescriptorExtractor extractor;

  cv::Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

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

  cv::Mat img_matches;
  cv::drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
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
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<cv::Point2f> scene_corners(4);

  cv::perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  cv::line( img_matches, scene_corners[0] + cv::Point2f( img_object.cols, 0), scene_corners[1] + cv::Point2f( img_object.cols, 0), cv::Scalar(0, 255, 0), 4 );
  cv::line( img_matches, scene_corners[1] + cv::Point2f( img_object.cols, 0), scene_corners[2] + cv::Point2f( img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
  cv::line( img_matches, scene_corners[2] + cv::Point2f( img_object.cols, 0), scene_corners[3] + cv::Point2f( img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
  //cv::line( img_matches, scene_corners[3] + cv::Point2f( img_object.cols, 0), scene_corners[0] + cv::Point2f( img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );


  if(side == "right")
  {
    returnPoints.up = scene_corners[1] ;//+ cv::Point2f( img_object.cols, 0);
    returnPoints.down = scene_corners[2] ;//+ cv::Point2f( img_object.cols, 0);
    //printf("Right: %d %d %d %d\n",returnPoints.up.x,returnPoints.up.y,returnPoints.down.x,returnPoints.down.y);
  }
  else if(side == "left")
  {
    returnPoints.down = scene_corners[3] ;//+ cv::Point2f( img_object.cols, 0);
    returnPoints.up = scene_corners[0] ;//+ cv::Point2f( img_object.cols, 0);
  //  printf("Left: %d %d %d %d\n",returnPoints.up.x,returnPoints.up.y,returnPoints.down.x,returnPoints.down.y);
  }

    //-- Show detected matches
  //cv::imshow( "Good Matches & Object detection" + side, img_matches );

  //cv::waitKey(0);



  return returnPoints;
  }


#endif