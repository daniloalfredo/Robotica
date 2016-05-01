#include "object_detector.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	srand(time(NULL));

	//Open the default camera
    VideoCapture cap(0); 
  
    if(!cap.isOpened())
        return -1;

	namedWindow("Object Detector", WINDOW_AUTOSIZE);
	namedWindow("Detection", WINDOW_AUTOSIZE);

    Mat frame;

	bool idle = true;
	bool exit_program = false;
	ObjectDetector objectDetector;

	//---------------------------------------------------------

	#pragma omp parallel sections
   	{ 
     	#pragma omp section
     	{ 
			while(!exit_program)
			{
				//Se apertar alguma tecla sai
				if(waitKey(30) >= 0)
					exit_program = true;       

				//Pega o próximo frame da câmera
        		cap >> frame; 

				imshow("Object Detector", frame);
			}
		}

     	#pragma omp section
     	{ 
			while(!exit_program)
			{
				if(idle)
				{
					idle = false;
					
					//Process frame and show detected object in another window 
					imshow("Detection", objectDetector.Detect(frame));
					
					idle = true;
				}
			}
		}
   	}
    
	return 0;
}
