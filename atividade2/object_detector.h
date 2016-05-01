#ifndef OBJECT_DETECTOR_H_INCLUDED
#define OBJECT_DETECTOR_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <omp.h>

class ObjectDetector
{
	private:
		//cv::SiftFeatureDetector detector;
		//std::vector<KeyPoint> keypoints;

	public:
		void Train();
		cv::Mat Detect(cv::Mat frame);
};

#endif
