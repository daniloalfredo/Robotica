#include <string>
#include <cstdio>
#include <cstdlib>

#include "ObjectDetector.h"

//Arquivos utilizados pelo programa
#define FILE_DICTIONARY "ini/detector_dictionary.yml"
#define FILE_DATABASE "ini/detector_database.ini"
#define FILE_PARAMS "ini/detector_params.ini"
#define FILE_SVM "ini/detector_svm.ini" 

int main(int argc, char** argv)
{
	ObjectDetector objectDetector;
	objectDetector.LoadParams(FILE_PARAMS);
	objectDetector.LoadObjects(FILE_DATABASE);
	objectDetector.Train();
	objectDetector.SaveDictionary(FILE_DICTIONARY);
	objectDetector.SaveSVM(FILE_SVM);

	return 0;
}
