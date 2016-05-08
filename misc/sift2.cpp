
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <bits/stdc++.h>
using namespace std;
using namespace cv;

const double THRESHOLD = 400;

/**
 * Calculate euclid distance
 */
double euclidDistance(Mat& vec1, Mat& vec2) 
{
  double sum = 0.0;
  int dim = vec1.cols;
  for (int i = 0; i < dim; i++) {
    sum += (vec1.at<uchar>(0,i) - vec2.at<uchar>(0,i)) * (vec1.at<uchar>(0,i) - vec2.at<uchar>(0,i));
  }
  return sqrt(sum);
}

/**
 * Find the index of nearest neighbor point from keypoints.
 */
int nearestNeighbor(Mat& vec, vector<KeyPoint>& keypoints, Mat& descriptors) {
  int neighbor = -1;
  double minDist = 1e6;
  
  for (int i = 0; i < descriptors.rows; i++) {
    KeyPoint pt = keypoints[i];
    Mat v = descriptors.row(i);
    double d = euclidDistance(vec, v);
    //printf("%d %f\n", v.cols, d);
    if (d < minDist) {
      minDist = d;
      neighbor = i;
    }
  }
  
  if (minDist < THRESHOLD) {
    return neighbor;
  }
  
  return -1;
}

/**
 * Find pairs of points with the smallest distace between them
 */
void findPairs(vector<KeyPoint>& keypoints1, Mat& descriptors1,
               vector<KeyPoint>& keypoints2, Mat& descriptors2,
               vector<Point2f>& srcPoints, vector<Point2f>& dstPoints) {
  for (int i = 0; i < descriptors1.rows; i++) {
    KeyPoint pt1 = keypoints1[i];
    Mat desc1 = descriptors1.row(i);
    int nn = nearestNeighbor(desc1, keypoints2, descriptors2);
    if (nn >= 0) {
      KeyPoint pt2 = keypoints2[nn];
      srcPoints.push_back(pt1.pt);
      dstPoints.push_back(pt2.pt);
    }
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    cerr << "Too few arguments" << endl;
    return -1;
  }
  
  const char* filename = argv[1];
  
  printf("load file:%s\n", filename);
  
  // initialize detector and extractor
  FeatureDetector* detector;
  detector = new SiftFeatureDetector(
                                     0, // nFeatures
                                     4, // nOctaveLayers
                                     0.04, // contrastThreshold
                                     10, //edgeThreshold
                                     1.6 //sigma
                                     );
  
  DescriptorExtractor* extractor;
  extractor = new SiftDescriptorExtractor();
  
  // Compute keypoints and descriptor from the source image in advance
  vector<KeyPoint> keypoints2;
  Mat descriptors2;
  
  Mat originalGrayImage = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  if (!originalGrayImage.data) {
    cerr << "gray image load error" << endl;
    return -1;
  }
  Mat originalColorImage = imread(filename, CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
  if (!originalColorImage.data) {
    cerr << "color image open error" << endl;
    return -1;
  }
  
  detector->detect(originalGrayImage, keypoints2);
  extractor->compute(originalGrayImage, keypoints2, descriptors2);
  printf("original image:%d keypoints are found.\n", (int)keypoints2.size());
  
  VideoCapture capture(0);
  capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  
  namedWindow("mywindow");
  Mat frame;
  while (1) {
    capture >> frame;
    
    // load gray scale image from camera
    Size size = frame.size();
    Mat grayFrame(size, CV_8UC1);
    cvtColor(frame, grayFrame, CV_BGR2GRAY);
    if (!grayFrame.data) {
      cerr << "cannot find image file1" << endl;
      exit(-1);
    }
    
    // Create a image for displaying mathing keypoints
    Size sz = Size(size.width + originalColorImage.size().width, size.height + originalColorImage.size().height);
    Mat matchingImage = Mat::zeros(sz, CV_8UC3);
    
    // Draw camera frame
    Mat roi1 = Mat(matchingImage, Rect(0, 0, size.width, size.height));
    frame.copyTo(roi1);
    // Draw original image
    Mat roi2 = Mat(matchingImage, Rect(size.width, size.height, originalColorImage.size().width, originalColorImage.size().height));
    originalColorImage.copyTo(roi2);
    
    vector<KeyPoint> keypoints1;
    Mat descriptors1;
    vector<DMatch> matches;
    
    // Detect keypoints
    detector->detect(grayFrame, keypoints1);
    extractor->compute(grayFrame, keypoints1, descriptors1);
    
    printf("image1:%zd keypoints are found.\n", keypoints1.size());
    
    for (int i=0; i<keypoints1.size(); i++){
      KeyPoint kp = keypoints1[i];
     // circle(matchingImage, kp.pt, cvRound(kp.size*0.25), Scalar(255,255,0), 1, 8, 0);
    }
    // Find nearest neighbor pairs
    vector<Point2f> srcPoints;
    vector<Point2f> dstPoints;
    findPairs(keypoints1, descriptors1, keypoints2, descriptors2, srcPoints, dstPoints);
    //printf("%zd keypoints are matched.\n", srcPoints.size());
    
    char text[256];
    //sprintf(text, "%zd/%zd keypoints matched.", srcPoints.size(), keypoints2.size());
    //putText(matchingImage, text, Point(0, cvRound(size.height + 30)), FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(0,0,255));
    
    // Draw line between nearest neighbor pairs
    /*
    for (int i = 0; i < (int)srcPoints.size(); ++i) {
      Point2f pt1 = srcPoints[i];
      Point2f pt2 = dstPoints[i];
      Point2f from = pt1;
      Point2f to   = Point(size.width + pt2.x, size.height + pt2.y);
      line(matchingImage, from, to, Scalar(0, 255, 255));
    }*/
    
    // Display mathing image
    imshow("mywindow", matchingImage);
    
    int c = waitKey(2);
    if (c == '\x1b')
      break;
  }
  
  return 0;
}