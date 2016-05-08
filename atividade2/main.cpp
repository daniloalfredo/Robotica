#include "object_detector.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	//Variáveis
	bool exit_program = false;
	ObjectDetector objectDetector(0.8);
	omp_lock_t framelock;
	omp_lock_t namelock;
	Mat frame(100, 100, CV_8UC3, Scalar(255, 0, 255));
	string object_name("Object Not found");
	VideoCapture cap(0);

	//Se não conseguir abrir a câmera encerra
	if(!cap.isOpened())
        return -1;
    
    //Inicializa locks    
    omp_init_lock(&framelock);
	omp_init_lock(&namelock);

	//Janela de exibição de frames
	namedWindow("Object Detector", WINDOW_AUTOSIZE);

	//Inicio do programa
	#pragma omp parallel sections
   	{ 
   		/*
   			Esta thread captura os frames da câmera e os exibe 
   			na janela do programa.
   		*/
     	#pragma omp section
     	{ 
			while(!exit_program)
			{
				//Pega próximo frame da câmera e faz cópia para exibição
				omp_set_lock(&framelock);
        		cap >> frame;
        		Mat frame_show = frame;
				omp_unset_lock(&framelock); 
				
				//Desenha nome do objeto na imagem do frame
				omp_set_lock(&namelock);
				putText(frame_show, object_name, Point(30, 30), FONT_ITALIC, 1, Scalar(0, 0, 0), 3, false);
				putText(frame_show, object_name, Point(32, 32), FONT_ITALIC, 1, Scalar(0, 255, 255), 3, false);
				omp_unset_lock(&namelock);

				//Mostra o frame na janela
				imshow("Object Detector", frame_show);

				//Sai do programa se alguma tecla for pressionada
				if(waitKey(30) >= 0)
					exit_program = true;
			}
		}

		/*
   			Esta thread processa os frames capturados e busca
   			os objetos na imagem.
   		*/
     	#pragma omp section
     	{ 
			while(!exit_program)
			{
				//Converte frame para preto e branco
				omp_set_lock(&framelock);
				Mat frame_bw;
				cvtColor(frame, frame_bw, CV_BGR2GRAY);
				omp_unset_lock(&framelock); 

				//Processa o frame e pega o nome do objeto encontrado
				string detector_name = objectDetector.Detect(frame_bw);
				
				//Copia a string do nome para a string de exibicao
				omp_set_lock(&namelock);
				object_name = detector_name;
				omp_unset_lock(&namelock);
			}
		}
   	}

	//Libera os locks
	omp_destroy_lock(&framelock);
	omp_destroy_lock(&namelock);
    
	return 0;
}
