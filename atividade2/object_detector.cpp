#include "object_detector.h"

void ObjectDetector::Train()
{

}

cv::Mat ObjectDetector::Detect(cv::Mat frame)
{
	cv::Mat detection(500, 500, CV_8UC3, cv::Scalar(rand()%256,rand()%256,rand()%256));
	return detection;
}
