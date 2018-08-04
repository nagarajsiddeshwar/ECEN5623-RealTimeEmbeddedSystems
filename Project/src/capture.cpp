#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include<pthread.h>
#include <X11/Xlib.h>
#include <sched.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>


#include <semaphore.h>


#include <string.h>

#include <syslog.h>
#include <math.h>
#include <sys/param.h>
#include <sys/time.h>
#include <errno.h>

using namespace cv;

const int alpha_slider_max = 255;
int alpha_slider;
double alpha;
Mat grayimage,thr;

void on_trackbar( int, void* )
{
 //alpha = (double) alpha_slider/alpha_slider_max ;

alpha = (double) alpha_slider;
 //imshow( "grayimage", grayimage);
}

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

     alpha_slider = 0;
    namedWindow("grayimage",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, grayimage, COLOR_BGR2GRAY);
       threshold(grayimage, thr,alpha, alpha_slider_max,THRESH_BINARY);   
        imshow("grayimage", grayimage);

waitKey(33);

        imshow("threshold output", thr);

waitKey(33);





 /// Create Windows
 namedWindow("grayimage", 1);

 /// Create Trackbars
 char TrackbarName[50];
 sprintf( TrackbarName, "Alpha x %d", alpha_slider_max );


 createTrackbar( TrackbarName, "threshold output", &alpha_slider, alpha_slider_max, on_trackbar );

 /// Show some stuff
// on_trackbar( alpha_slider, 0 );

 /// Wait until user press some key
 
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
