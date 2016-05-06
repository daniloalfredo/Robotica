#ifndef OBJECT_DETECTOR_H_INCLUDED
#define OBJECT_DETECTOR_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <omp.h>

using namespace std;
using namespace cv;

#define NUM_IMAGES 15

class ObjectDetector
{
	private:
				

	public:
		ObjectDetector();
		void Train();
		cv::Mat Detect(cv::Mat frame);
};

#endif
