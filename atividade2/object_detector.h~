#ifndef OBJECT_DETECTOR_H_INCLUDED
#define OBJECT_DETECTOR_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <string>
#include <omp.h>

using namespace std;
using namespace cv;

class Object
{
	private:
		int id;
		string name;
		vector<string> image_filenames;
		
	public:
		Object() { }
		Object(int id, string name, vector<string> image_filenames) { this->id = id; this->name = name; this->image_filenames = image_filenames; } 
		void SetID(int id) { this->id = id; }
		void SetName(string name) { this->name = name; }
		void InsertImageFilename(string image_filename) { image_filenames.push_back(image_filename); }
		
		int GetID() { return id; }
		string GetName() { return name; }
		vector<string> GetFilenames() { return image_filenames; }
};

class ObjectDetector
{
	private:
		vector<Object> objects;
		Mat dictionary;
		Mat labels;
		CvSVM svm;
		
		double confidence_threshold;
		
		void Init(double confidence);
		void Train();
		void LoadObjects();
		Mat ComputeHistogram(Mat image);	

	public:
		ObjectDetector();
		ObjectDetector(double confidence_threshold);
		string Detect(Mat frame);
};

#endif
