#include "object_detector.h"
#include "timer.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	//Variáveis
	bool exit_program = false;
	ObjectDetector objectDetector;
	omp_lock_t framelock;
	omp_lock_t namelock;
	Mat frame(1, 1, CV_32FC1, Scalar(255, 255, 255));
	string object_name("Object Not found");
	VideoCapture cap(0);
	Timer timer_fps;
	int cont_frames = 0;
	char frame_rate[20] = "30 fps";

	//Se não conseguir abrir a câmera encerra
	if(!cap.isOpened())
        return -1;
    
    //Inicializa locks    
    omp_init_lock(&framelock);
	omp_init_lock(&namelock);

	//Janela de exibição de frames
	namedWindow("Object Detector", WINDOW_AUTOSIZE);
	
	//Inicia timer de fps
	timer_fps.Start();

	//Inicio do programa
	#pragma omp parallel sections
   	{ 
   		/*
   			Esta thread captura os frames da câmera e os exibe 
   			na janela do programa contando também a taxa de quadros
   			por segundo.
   		*/
     	#pragma omp section
     	{ 
			while(!exit_program)
			{
				//Atualiza contador de fps
				if(timer_fps.GetTimeMsec() >= 1000)
				{
					sprintf(frame_rate, "%d fps", cont_frames);
					cont_frames = 0;
					timer_fps.Start();
				}
			
				//Pega próximo frame da câmera e faz cópia para exibição
				omp_set_lock(&framelock);
        		cap >> frame;
        		Mat frame_show = frame;
				omp_unset_lock(&framelock); 
				
				//Desenha nome do objeto na imagem do frame
				omp_set_lock(&namelock);
				putText(frame_show, object_name, Point(10, 30), FONT_ITALIC, 1, Scalar(0, 0, 0), 3, false);
				putText(frame_show, object_name, Point(12, 32), FONT_ITALIC, 1, Scalar(0, 255, 255), 3, false);
				omp_unset_lock(&namelock);
				
				//Desenha framerate na imagem do frame
				int x = frame_show.cols-120;
				int y = frame_show.rows-20;
				putText(frame_show, frame_rate, Point(x, y), FONT_ITALIC, 1, Scalar(0, 0, 0), 3, false);
				putText(frame_show, frame_rate, Point(x+2, y+2), FONT_ITALIC, 1, Scalar(255, 255, 0), 3, false);

				//Mostra o frame na janela
				imshow("Object Detector", frame_show);

				//Sai do programa se alguma tecla for pressionada
				if(waitKey(30) >= 0)
					exit_program = true;
					
				//Conta os frames que se passaram
				cont_frames++;
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
				//Se certifica de que frame já foi inicializado com a câmera
				if(frame.rows > 1)
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
   	}

	//Libera os locks
	omp_destroy_lock(&framelock);
	omp_destroy_lock(&namelock);
    
	return 0;
}
