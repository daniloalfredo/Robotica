#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <bits/stdc++.h>
//using namespace std;
// using namespace cv;


int main(int argc, char* argv[])
{
    cv::VideoCapture cap(0); 

    if (!cap.isOpened())
    {
        std::cout << "ERROR: Cannot open the video file" << std::endl;
        return -1;
    }

   cv::namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); 

   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

   std::cout << "Frame Size = " << dWidth << "x" << dHeight << std::endl;

  cv::Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
 
 cv::VideoWriter oVideoWriter ("video.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true); //initialize the VideoWriter object 

   if ( !oVideoWriter.isOpened() ) 
   {
        std::cout << "ERROR: Failed to write the video" << std::endl;
        return -1;
   }

    while (1)
    {

        cv::Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

        if (!bSuccess) //if not success, break loop
       {
             std::cout << "ERROR: Cannot read a frame from video file" << std::endl;
             break;
        }

         oVideoWriter.write(frame); //writer the frame into the file

        cv::imshow("MyVideo", frame); //show the frame in "MyVideo" window

        if (cv::waitKey(10) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            std::cout << "esc key is pressed by user" << std::endl;
            break; 
       }
    }

    return 0;

}