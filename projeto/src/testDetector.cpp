#include <string>
#include <cstdio>
#include <cstdlib>

#include "Utils.h"
#include "ObjectDetector.h"

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

	//Captura imagens e detecta objetos
	for(int i = 0; i < 5 && !exit_program; i++)
	{
		//Captura frame
		cap >> frame;

		//Converte para preto e branco
		Mat frame_bw;
		cvtColor(frame, frame_bw, CV_BGR2GRAY);

		//Classifica
		objectDetector.Detect(frame_bw, objectName);

		//Printa objeto encontrado na tela
		printf("Frame %d: %s\n", i, objectName);

		//Sai do programa se alguma tecla for pressionada
		if(waitKey(30) >= 0)
			exit_program = true;
	}

	return 0;
}
