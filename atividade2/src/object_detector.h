#ifndef OBJECT_DETECTOR_H_INCLUDED
#define OBJECT_DETECTOR_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <algorithm>
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
		
		Mat validation_image;
		vector<KeyPoint> validation_keypoints;
		Mat validation_descriptors;
		
	public:
		Object(string name, vector<string> image_filenames, int descriptor_extractor)
		{ 
			this->name = name; 
			this->image_filenames = image_filenames;
			this->main_image = imread(image_filenames[0], CV_LOAD_IMAGE_GRAYSCALE);
			this->validation_image = imread(image_filenames[(int)image_filenames.size()-1], CV_LOAD_IMAGE_GRAYSCALE);
			
			if(descriptor_extractor == 0)
			{
				SiftDescriptorExtractor detector_sift;
				detector_sift.detect(main_image, keypoints);
	  			detector_sift.compute(main_image, keypoints, descriptors);
	  			detector_sift.detect(validation_image, validation_keypoints);
	  			detector_sift.compute(validation_image, validation_keypoints, validation_descriptors);
	  		}
  			
  			else
  			{
  				SurfDescriptorExtractor detector_surf;
				detector_surf.detect(main_image, keypoints);
	  			detector_surf.compute(main_image, keypoints, descriptors);
	  			detector_surf.detect(validation_image, validation_keypoints);
	  			detector_surf.compute(validation_image, validation_keypoints, validation_descriptors);
  			}
		}
		 
		void InsertImageFilename(string image_filename) { image_filenames.push_back(image_filename); }
		
		string GetName() { return name; }
		vector<string> GetFilenames() { return image_filenames; }
		string GetFilename(int i) { return image_filenames[i]; }
		int GetNumImages() { return (int) image_filenames.size(); }
		Mat GetMainImage() { return main_image; }
		vector<KeyPoint> GetKeypoints() { return keypoints; }
		Mat GetDescriptors() { return descriptors; }
		Mat GetValidationImage() { return validation_image; }
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
		int descriptor_extractor;
		int dictionary_size;
		int blur_size;
		int svm_kernel_type;
		int svm_degree;
		int svm_gamma;
		bool use_advanced_training;
		int num_svms_for_advanced_training;
		
		//Funções auxiliares
		void Init();
		void LoadDetectorParams();
		void LoadObjects();
		void Train();
		void TrainAdvanced();
		void SaveKeypointImageLog(Mat image, vector<KeyPoint>keypoints, unsigned int i, unsigned int j);
		Mat ComputeHistogram(Mat image);
		void FindCenter(Mat frame, Point2f* center_pos, int object_class);	
		
		vector<string> GetSubsetOfImages(int obj_id);
		int ValidateSVM();

	public:
		ObjectDetector();
		string Detect(Mat frame, Point2f* center_pos);
};

#endif
