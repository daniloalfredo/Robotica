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

using namespace std;
using namespace cv;

//Arquivos utilizados pelo programa
#define FILE_DICTIONARY "ini/dictionary.yml"
#define FILE_DATABASE "ini/database.ini"
#define FILE_PARAMS "ini/params.ini"
#define FILE_SVM "ini/svm.ini" 

class Object
{
	private:
		string name;
		vector<string> image_filenames;
		
		Mat main_image;
		vector<KeyPoint> keypoints;
		Mat descriptors;
		
	public:
		Object(string name, vector<string> image_filenames)
		{ 
			this->name = name; 
			this->image_filenames = image_filenames;
			this->main_image = imread(image_filenames[0], CV_LOAD_IMAGE_GRAYSCALE);
			
			SiftDescriptorExtractor detector;
			detector.detect(main_image, keypoints);
  			detector.compute(main_image, keypoints, descriptors);
		}
		 
		void InsertImageFilename(string image_filename) { image_filenames.push_back(image_filename); }
		
		string GetName() { return name; }
		vector<string> GetFilenames() { return image_filenames; }
		Mat GetMainImage() { return main_image; }
		vector<KeyPoint> GetKeypoints() { return keypoints; }
		Mat GetDescriptors() { return descriptors; }
};

class ObjectDetector
{
	private:
		vector<Object> objects;
		Mat dictionary;
		Mat labels;
		CvSVM svm;
		
		vector<KeyPoint> keypoints;
		
		//Parâmetros de aprendizado
		double confidence_threshold;
		int dictionary_size;
		
		//Funções auxiliares
		void Init();
		void LoadDetectorParams();
		void LoadObjects();
		void Train();
		Mat ComputeHistogram(Mat image);
		void FindCenter(Mat frame, Point2f* center_pos, int object_class);	

	public:
		ObjectDetector();
		string Detect(Mat frame, Point2f* center_pos);
};

#endif
