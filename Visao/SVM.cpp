#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <cstdio>

using namespace cv;
using namespace std;

int main ()
{
	CvSVM svm;
	
	//Preparar matriz com padrões de treino
	Mat training_matrix(6, 3, CV_32FC1); //6 padrões, 3 atributos cada
	
	training_matrix.at<float>(0, 0) = 0.80; training_matrix.at<float>(0, 1) = 0.30; training_matrix.at<float>(0, 2) = 0.10;
	training_matrix.at<float>(1, 0) = 0.70; training_matrix.at<float>(1, 1) = 0.25; training_matrix.at<float>(1, 2) = 0.12;
	training_matrix.at<float>(2, 0) = 0.78; training_matrix.at<float>(2, 1) = 0.22; training_matrix.at<float>(2, 2) = 0.16;
	training_matrix.at<float>(3, 0) = 0.20; training_matrix.at<float>(3, 1) = 0.75; training_matrix.at<float>(3, 2) = 0.60;
	training_matrix.at<float>(4, 0) = 0.15; training_matrix.at<float>(4, 1) = 0.78; training_matrix.at<float>(4, 2) = 0.55;
	training_matrix.at<float>(5, 0) = 0.19; training_matrix.at<float>(5, 1) = 0.70; training_matrix.at<float>(5, 2) = 0.50;
	
	//Preparar labels com outputs conhecidos para os padrões de treino
	Mat labels(6, 1, CV_32FC1); //6 padroes de treino
	labels.at<float>(0, 0) = 1; labels.at<float>(1, 0) = 1; labels.at<float>(2, 0) = 1; //classe 1
	labels.at<float>(3, 0) = 2; labels.at<float>(4, 0) = 2; labels.at<float>(5, 0) = 2; //classe 2
	
	//float labels1[6] = {1.0, 1.0, 1.0, 2.0, 2.0, 2.0};
    //Mat labelsMat(6, 1, CV_32FC1, labels1);
    //float trainingData[6][3] = { {80,30,10}, {70,25,12}, {78,22,16}, {20,75,60}, {15,78,55}, {19,70,50} };
    //Mat trainingDataMat(6, 3, CV_32FC1, trainingData);

	//Treina o SVM	
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::RBF;
	params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	
	svm.train(training_matrix, labels, Mat(), Mat(), params);
	//svm.train(trainingDataMat, labelsMat, Mat(), Mat(), params);

	//Classifica novo padrão
	Mat new_row(3, 1, CV_32FC1); //3 atributos
	//new_row.at<float>(0, 0) = 0.73; new_row.at<float>(1, 0) = 0.27; new_row.at<float>(2, 0) = 0.11; //um padrão da classe 1
	new_row.at<float>(0, 0) = 0.18; new_row.at<float>(1, 0) = 0.69; new_row.at<float>(2, 0) = 0.54; //um padrão da classe 2
	
	printf("prediction: %lf\n", svm.predict(new_row));
}
