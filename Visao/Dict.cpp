#include <opencv2/opencv.hpp>
#include "homography.hpp"
#include <string.h>	
using namespace cv;
using namespace std;

  
 int main(int, char**)
 {
 	//Step 1 - Obtain the set of bags of features.

	//to store the input file names
	char * filename = new char[100];        
	char * name = new char[100];
	//to store the current input image
	Mat input;    

	//To store the keypoints that will be extracted by SIFT
	vector<KeyPoint> keypoints;
	//To store the SIFT descriptor of current image
	Mat descriptor;
	//To store all the descriptors that are extracted from all the images.
	Mat featuresUnclustered;
	//The SIFT feature extractor and descriptor
	SiftDescriptorExtractor detector;  
	//List of image filenames
	FILE* img = fopen("imagelist.txt", "r"); 

	while (fscanf(img, "%s", name) > 0)
	{
		strcpy(filename, "Imagens/");
		strcat(filename, name);
		input = imread(filename, CV_LOAD_IMAGE_GRAYSCALE); //Load as grayscale                
	    namedWindow("Display Image", WINDOW_AUTOSIZE );
    	imshow("Display Image", input);

    	waitKey(0);
	    //detect feature points
	    detector.detect(input, keypoints);
	    //compute the descriptors for each keypoint
	    detector.compute(input, keypoints,descriptor);        
	    //put the all feature descriptors in a single Mat object 
	    featuresUnclustered.push_back(descriptor);        
	}

	//Construct BOWKMeansTrainer
	//the number of bags
	int dictionarySize=200;
	//define Term Criteria
	TermCriteria tc(CV_TERMCRIT_ITER,100,0.001);
	//retries number
	int retries=1;
	//necessary flags
	int flags=KMEANS_PP_CENTERS;
	//Create the BoW (or BoF) trainer
	BOWKMeansTrainer bowTrainer(dictionarySize,tc,retries,flags);
	//cluster the feature vectors
	Mat dictionary=bowTrainer.cluster(featuresUnclustered);    
	//store the vocabulary
	FileStorage fs("dictionary.yml", FileStorage::WRITE);
	fs << "vocabulary" << dictionary;
	fs.release();
 }