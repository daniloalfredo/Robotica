#include "object_detector.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	srand(time(NULL));

	//Variables
	bool exit_program = false;
	Mat frame(100, 100, CV_8UC3, Scalar(255, 0, 255));
	ObjectDetector objectDetector;
	VideoCapture cap(0); //default camera
	omp_lock_t writelock;
	omp_init_lock(&writelock);

	if(!cap.isOpened())
        return -1;

	namedWindow("Object Detector", WINDOW_AUTOSIZE);
	namedWindow("Detection", WINDOW_AUTOSIZE);

	//Do training of the database
	objectDetector.Train();

	//---------------------------------------------------------

	#pragma omp parallel sections
   	{ 
     	#pragma omp section
     	{ 
			while(!exit_program)
			{
				//Get next frame from camera
				omp_set_lock(&writelock);
        		cap >> frame;
				omp_unset_lock(&writelock); 

				//Show frame on window
				imshow("Object Detector", frame);

				//Exit if any key pressed
				if(waitKey(30) >= 0)
					exit_program = true;
			}
		}

     	#pragma omp section
     	{ 
			while(!exit_program)
			{
				//Convert frame to black and white
				Mat frame_bw;

				omp_set_lock(&writelock);
				cvtColor(frame, frame_bw, CV_BGR2GRAY);
				omp_unset_lock(&writelock); 

				//Process frame and show detected object in another window 
				imshow("Detection", objectDetector.Detect(frame_bw));

				cvWaitKey(10);
			}
		}
   	}

	omp_destroy_lock(&writelock);
    
	return 0;
}
