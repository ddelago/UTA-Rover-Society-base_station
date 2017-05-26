#include <cv.h>
#include <highgui.h>

int main()
{
        //display the original image
        IplImage* img = cvLoadImage("nasa.jpg");
        cvNamedWindow("MyWindow");
        cvShowImage("MyWindow", img);

        //erode and display the eroded image
        cvErode(img, img, 0, 3);
        cvNamedWindow("Eroded");
        cvShowImage("Eroded", img);
        
        cvWaitKey(0);
        
        //cleaning up
        cvDestroyWindow("MyWindow");
        cvDestroyWindow("Eroded");
        cvReleaseImage(&img);
        
        return 0;
} 