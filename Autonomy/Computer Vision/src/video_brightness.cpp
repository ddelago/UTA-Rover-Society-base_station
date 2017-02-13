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

    namedWindow("Original Video",CV_WINDOW_AUTOSIZE); //create a window called "Original Video"
    namedWindow("Brightness Increased",CV_WINDOW_AUTOSIZE); //create a window called "Brightness Increased"
    namedWindow("Brightness Decreased",CV_WINDOW_AUTOSIZE); //create a window called "Brightness Decreased"

    while(1)
    {
           Mat frame;

           bool bSuccess = cap.read(frame); // read a new frame from video

            if (!bSuccess) //if not success, break loop
           {
                        cout << "Cannot read the frame from video file" << endl;
                       break;
           }

            Mat imgH = frame + Scalar(50, 50, 50); //increase the brightness by 75 units

            Mat imgL = frame + Scalar(-50, -50, -50); //decrease the brightness by 75 units

           imshow("Original Video", frame); //show the frame in "Original Video" window
           imshow("Brightness Increased", imgH); //show the frame of which brightness increased
           imshow("Brightness Decreased", imgL); //show the frame of which brightness decreased

           if (waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
           {
                   cout << "esc key is pressed by user" << endl; 
                   break; 
           }
    }

    return 0;

}