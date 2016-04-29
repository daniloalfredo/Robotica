#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>


int main(int argc, char* argv[])
{
    cv::VideoCapture cap("video.avi"); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        std::cout << "ERROR: Cannot open the video file" << std::endl;
        return -1;
    }
    
    cv::Mat frame;
    cv::namedWindow("Testando");
    double rate = cap.get(CV_CAP_PROP_FPS);
    int delay = 1000/rate;


    while(1)
    {
       if(!cap.read(frame)) break;
       cv::imshow("Testando",frame);


        if(cv::waitKey(delay)>=0)
            break;

    }

    cap.release();


    return 0;

}