#include "object_detector.h"
#include "timer.h"

#define USE_OPENMP

#ifdef USE_OPENMP 
#include <omp.h>
#endif

using namespace cv;
using namespace std;

//Pra a webcam não travar se interromper a captura
volatile int quit_signal = 0;
#ifdef __unix__
#include <signal.h>
extern "C" void quit_signal_handler(int signum)
{
	if (quit_signal != 0) 
		exit(0);
	quit_signal = 1;
}
#endif

//Variáveis
bool exit_program = false;
ObjectDetector objectDetector;
Mat frame(1, 1, CV_32FC1);
string object_name("Object Not found");
Point2f object_center(-1, -1);
VideoCapture cap(0);
Timer timer_fps;
int cont_frames = 0;
char frame_rate[20] = "30 fps";

#ifdef USE_OPENMP 
omp_lock_t framelock;
omp_lock_t namelock;
#endif


/*
   	Esta thread captura os frames da câmera e os exibe 
   	na janela do programa contando também a taxa de quadros
   	por segundo.
*/
void thread_capture()
{
	//Atualiza contador de fps
	if(timer_fps.GetTimeSec() >= 1)
	{
		sprintf(frame_rate, "%d fps", cont_frames);
		cont_frames = 0;
		timer_fps.Start();
	}
			
	//Pega próximo frame da câmera e faz cópia para exibição
	#ifdef USE_OPENMP 
	omp_set_lock(&framelock);
	#endif
	
	cap >> frame;
	Mat frame_show = frame.clone();
	
	#ifdef USE_OPENMP
	omp_unset_lock(&framelock);
	#endif 
	
	//Fecha o programa se apertar ctrl+C no terminal
	if(quit_signal)
		exit(0);
				
	//Desenha nome do objeto e centro na imagem do frame
	#ifdef USE_OPENMP
	omp_set_lock(&namelock);
	#endif
	
	putText(frame_show, object_name, Point(10, 30), FONT_ITALIC, 1, Scalar(0, 0, 0), 3, false);
	putText(frame_show, object_name, Point(12, 32), FONT_ITALIC, 1, Scalar(0, 255, 255), 3, false);
				
	if(object_center.x >= 0)
		circle(frame_show, object_center, 40, Scalar(0, 0, 255), 2, 8, 0);
	
	#ifdef USE_OPENMP			
	omp_unset_lock(&namelock);
	#endif
				
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


/*
   	Esta thread processa os frames capturados e busca
   	os objetos na imagem.
*/
void thread_detection()
{
	//Se certifica de que frame já foi inicializado com a câmera
	if(frame.rows > 1)
	{
		//Converte frame para preto e branco
		#ifdef USE_OPENMP
		omp_set_lock(&framelock);
		#endif
		
		Mat frame_bw;
		cvtColor(frame, frame_bw, CV_BGR2GRAY);
		
		#ifdef USE_OPENMP
		omp_unset_lock(&framelock);
		#endif 

		//Processa o frame e pega o nome do objeto encontrado
		Point2f detector_center;
		string detector_name = objectDetector.Detect(frame_bw, &detector_center);
				
		//Copia a string do nome para a string de exibicao
		#ifdef USE_OPENMP
		omp_set_lock(&namelock);
		#endif
		
		object_name = detector_name;
		object_center = detector_center;
		
		#ifdef USE_OPENMP
		omp_unset_lock(&namelock);
		#endif
	}
}

int main(int argc, char** argv)
{
	#ifdef __unix__
   	signal(SIGINT,quit_signal_handler); // listen for ctrl-C
	#endif

	//Se não conseguir abrir a câmera encerra
	if(!cap.isOpened())
        return -1;
        
    //Seta o background
    Mat background;
    cap >> background;
    objectDetector.SetBackground(background);
    
    //Inicializa locks
    #ifdef USE_OPENMP    
    omp_init_lock(&framelock);
	omp_init_lock(&namelock);
	#endif

	//Janela de exibição de frames
	namedWindow("Object Detector", CV_WINDOW_AUTOSIZE);
	
	//Inicia timer de fps
	timer_fps.Start();

	//Inicio do programa
	#ifdef USE_OPENMP 
		#pragma omp parallel sections
	   	{ 
		 	#pragma omp section
		 	{ 
				while(!exit_program)
					thread_capture();
			}

		 	#pragma omp section
		 	{ 
				while(!exit_program)
					thread_detection();
			}
	   	}
	   	
	#else
		while(!exit_program)
		{
			thread_capture();
			thread_detection();
		}
	#endif 

	//Libera os locks
	#ifdef USE_OPENMP 
	omp_destroy_lock(&framelock);
	omp_destroy_lock(&namelock);
	#endif
    
	return 0;
}
