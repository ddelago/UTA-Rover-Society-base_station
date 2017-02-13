#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    VideoCapture cap(0); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    //create windows
    namedWindow("Original Video",CV_WINDOW_AUTOSIZE); 
    namedWindow("Contrast Increased",CV_WINDOW_AUTOSIZE); 
    namedWindow("Contrast Decreased",CV_WINDOW_AUTOSIZE); 

    while (1)
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
                 cout << "Cannot read the frame from video file" << endl;
                 break;
        }

        Mat imgH;
        frame.convertTo(imgH, -1, 2, 0); //increase the contrast (double)

        Mat imgL;
        frame.convertTo(imgL, -1, 0.5, 0); //decrease the contrast (halve)

        //show the image
        imshow("Original Video", frame); 
        imshow("Contrast Increased", imgH); 
        imshow("Contrast Decreased", imgL); 

       if (waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
        {
                cout << "esc key is pressed by user" << endl; 
                break; 
        }
    }
    return 0;
}