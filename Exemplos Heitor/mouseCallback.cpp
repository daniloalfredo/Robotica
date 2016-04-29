#include "opencv2/highgui/highgui.hpp"
#include <iostream>
 
using namespace std;
using namespace cv;

Mat img;
Point pt1,pt2,pt3,pt4;
int T = 0;
 
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        if(T == 0)
        {
        	pt1.x = x;
        	pt1.y = y;
        	rectangle( img, pt1, Point( pt1.x+10, pt1.y+10), Scalar( 0, 0, 255 ), 3, CV_FILLED );
        	T++;
        } 
        else if(T==1)
		{
			pt2.x = x;
        	pt2.y = y;
			rectangle( img, pt2, Point( pt2.x+10, pt2.y+10), Scalar( 0, 0, 255 ), 3, CV_FILLED );
   			T++;
   		}
   		else if(T==2)
   		{
   			pt3.x = x;
        	pt3.y = y;
        	rectangle( img, pt3, Point( pt3.x+10, pt3.y+10), Scalar( 0, 0, 255 ), 3, CV_FILLED );
			T++;	
   		}
   		else if(T == 3)
   		{
   			pt4.x = x;
        	pt4.y = y;
        	rectangle( img, pt4, Point( pt4.x+10, pt4.y+10), Scalar( 0, 0, 255 ), 3, CV_FILLED );
			T++;
   		}
   		else
   		{
   			T=0; // Eh so pra salvar os 4 pontos, e resetar toda vez...
   		}
   		imshow("ImageDisplay", img);
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
    }    
}
 
int main()
{
    // Read image from file 
    img = imread("image.jpg");
 
    //if fail to read the image
    if ( img.empty() ) 
    { 
        cout << "Error loading the image" << endl;
        return -1; 
    }
 
    //Create a window
    namedWindow("ImageDisplay", 1);
 
    //set the callback function for any mouse event
    setMouseCallback("ImageDisplay", CallBackFunc, NULL);
 
    //show the image
    imshow("ImageDisplay", img);
 
    // Wait until user press some key
    waitKey(0);
 
    return 0;
 
}