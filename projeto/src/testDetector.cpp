#include <string>
#include <cstdio>
#include <cstdlib>

#include "Utils.h"
#include "ObjectDetector.h"
#include "KBAsync.h"

//Arquivos utilizados pelo programa
#define FILE_DICTIONARY "ini/detector_dictionary.yml"
#define FILE_DATABASE "ini/detector_database.ini"
#define FILE_PARAMS "ini/detector_params.ini"
#define FILE_SVM "ini/detector_svm.ini" 

int main(int argc, char** argv)
{
	bool exit_program = false;
	Mat frame(1, 1, CV_32FC1);
	char objectName[100];

	//Inicializa a captura
	VideoCapture cap(0);
	if(!cap.isOpened())
	{
		printf("Error. Could not open VideoCapture\n");
        return -1;
	}

	//Inicializa o detector de objetos
	ObjectDetector objectDetector;
	objectDetector.LoadParams(FILE_PARAMS);
	objectDetector.LoadObjects(FILE_DATABASE);

	if(objectDetector.LoadDictionary(FILE_DICTIONARY))
	{
		objectDetector.LoadSVM(FILE_SVM);
	}
	
	else
	{
		objectDetector.Train();
		objectDetector.SaveDictionary(FILE_DICTIONARY);
		objectDetector.SaveSVM(FILE_SVM);
	}

	printf("Press 'q' to exit.\n");
	KBAsync kb;

	//Captura imagens e detecta objetos
	while(!exit_program)
	{
		//Sai do programa se q for pressionado
		if(kb.getKey() == 'q')
			exit_program = true;

		//Captura frame
		cap >> frame;

		//Classifica
		objectDetector.Detect(frame, objectName);

		//Printa objeto encontrado na tela
		printf("Object: %s\n", objectName);
	}

	return 0;
}
