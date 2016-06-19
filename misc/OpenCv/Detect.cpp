#include <opencv2/opencv.hpp>
#include "homography.hpp"
#include <string.h>	
using namespace cv;
using namespace std;

int main (){

//prepare BOW descriptor extractor from the dictionary    
    Mat dictionary; 
    FileStorage fs("dictionary.yml", FileStorage::READ);
    fs["vocabulary"] >> dictionary;
    fs.release();    
    
    //create a nearest neighbor matcher
    Ptr<DescriptorMatcher> matcher(new FlannBasedMatcher);
    //create Surf feature point extracter
    Ptr<FeatureDetector> detector(new SurfFeatureDetector());
    //create Surf descriptor extractor
    Ptr<DescriptorExtractor> extractor(new SurfDescriptorExtractor);    
    //create BoF (or BoW) descriptor extractor
    BOWImgDescriptorExtractor bowDE(extractor,matcher);
    //Set the dictionary with the vocabulary we created in the first step
    bowDE.setVocabulary(dictionary);
 
    //To store the image file name
    char * filename = new char[100];
    //To store the image tag name - only for save the descriptor in a file
    char * imageTag = new char[10];
 
    //open the file to write the resultant descriptor
    FileStorage fs1("descriptor.yml", FileStorage::WRITE);    
    
    //the image file with the location. change it according to your image file location
    sprintf(filename,"obj1_01.jpg");        
    //read the image
    Mat img=imread(filename,CV_LOAD_IMAGE_GRAYSCALE);        
    //To store the keypoints that will be extracted by Surf
    vector<KeyPoint> keypoints;        
    //Detect Surf keypoints (or feature points)
    detector->detect(img,keypoints);
    //To store the BoW (or BoF) representation of the image
    Mat bowDescriptor;        
    //extract BoW (or BoF) descriptor from given image
    bowDE.compute(img,keypoints,bowDescriptor);
 
    //prepare the yml (some what similar to xml) file
    sprintf(imageTag,"img1");            
    //write the new BoF descriptor to the file
    fs1 << imageTag << bowDescriptor;        
 
    //You may use this descriptor for classifying the image.
            
    //release the file storage
    fs1.release();
}