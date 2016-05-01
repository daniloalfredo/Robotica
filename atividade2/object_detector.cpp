#include "object_detector.h"

void ObjectDetector::Train()
{

}

cv::Mat ObjectDetector::Detect(cv::Mat frame)
{
	//Do SIFT hard work...
	for(int i = 0; i < 100000000; i++);

	//Return image of the detected object
	cv::Mat detection(500, 500, CV_8UC3, cv::Scalar(rand()%256,rand()%256,rand()%256));
	
	return detection;
}
