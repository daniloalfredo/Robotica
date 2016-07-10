#include "ColorDetector.h"

std::string ColorDetector::Detect(cv::Mat img)
{
    cv::Mat red, blue, white, yellow, black, green, skinColor;
 
    int qtdRed = 0;
    int qtdBlue = 0;
    int qtdWhite = 0;
    int qtdYellow = 0;
    int qtdBlack = 0;
    int qtdGreen = 0;
    int qtdSkinColor = 0;
 
    cvtColor(img,img, CV_BGR2HSV);
    inRange(img, cv::Scalar(110, 60, 105), cv::Scalar(179, 255, 255), red);
    inRange(img, cv::Scalar(91, 57, 96), cv::Scalar(128, 255, 214), blue);
    inRange(img, cv::Scalar(40, 0, 148), cv::Scalar(105, 62, 248), white);
    inRange(img, cv::Scalar(14, 103, 85), cv::Scalar(26, 255, 239), yellow);
    inRange(img, cv::Scalar(103, 18, 70), cv::Scalar(122, 60, 92), black);
    inRange(img, cv::Scalar(30, 32, 88), cv::Scalar(71, 108, 181), green);
    inRange(img, cv::Scalar(151, 0, 61), cv::Scalar(164, 63, 236), skinColor);
 
    qtdRed = countNonZero(red);
    qtdBlue = countNonZero(blue);
    qtdWhite = countNonZero(white);
    qtdYellow = countNonZero(yellow);
    qtdBlack = countNonZero(black);
    qtdGreen = countNonZero(green);
    qtdSkinColor = countNonZero(skinColor);
 
    if ( qtdYellow >= 350 && qtdYellow <= 45000 )
        return std::string("Charlie Brown");
    else if ( qtdRed >= 2400 && qtdRed <= 20000 && qtdWhite >= 1000 && qtdGreen > 0 && qtdSkinColor <= 9000 )
        return std::string("Snoop");
    else if ( qtdBlue >= 1000 && qtdWhite >= 1000 && qtdYellow <= 100 )
        return std::string("Lucy");
    else if ( qtdRed >= 200 && qtdRed <= 5000 && qtdSkinColor >= 1000 )
        return std::string("Linus");
   
    return std::string("Nao detectado");
}