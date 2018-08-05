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
#include "opencv2/objdetect/objdetect.hpp"

#include <semaphore.h>


#include <string.h>

#include <syslog.h>
#include <math.h>
#include <sys/param.h>
#include <sys/time.h>
#include <errno.h>

using namespace cv;

const int alpha_slider_max = 255;
int alpha_slider_red,alpha_slider_green, alpha_slider_blue;
double alpha_red,alpha_green,alpha_blue;
Mat grayimage, grayimage_red,grayimage_green,grayimage_blue,thr_red,thr_green,thr_blue,thr,bgr[3],merged_image;

//vector <Mat> bgr[3];

void on_trackbar_red( int, void* )
{
 //alpha = (double) alpha_slider/alpha_slider_max ;

alpha_red = (double) alpha_slider_red;
 //imshow( "grayimage", grayimage);
}

void on_trackbar_green( int, void* )
{
 //alpha = (double) alpha_slider/alpha_slider_max ;

alpha_green = (double) alpha_slider_green;
 //imshow( "grayimage", grayimage);
}

void on_trackbar_blue( int, void* )
{
 //alpha = (double) alpha_slider/alpha_slider_max ;

alpha_blue = (double) alpha_slider_blue;
 //imshow( "grayimage", grayimage);
}


int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

     alpha_slider_red = 0;
alpha_slider_green = 0;
alpha_slider_blue = 0;
    namedWindow("grayimage",1);
merged_image = Mat::zeros(Size(grayimage.rows, grayimage.cols), CV_8UC3);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, grayimage, COLOR_BGR2GRAY);
       split(frame, bgr);  // blue channel 

       threshold(bgr[0], thr_red,alpha_red, alpha_slider_max,THRESH_BINARY);   
       threshold(bgr[1], thr_green,alpha_green, alpha_slider_max,THRESH_BINARY);   
       threshold(bgr[2], thr_blue,alpha_blue, alpha_slider_max,THRESH_BINARY);   
    //    imshow("grayimage", grayimage);

waitKey(33);

        imshow("threshold output for red", thr_red);

waitKey(33);


        imshow("threshold output for green", thr_green);

waitKey(33);

        imshow("threshold output for blue", thr_blue);

waitKey(33);

 /// Create Windows
 namedWindow("grayimage", 1);

 /// Create Trackbars
 char TrackbarName_red[50],TrackbarName_green[50],TrackbarName_blue[50];
 sprintf( TrackbarName_red, "Alpha red x %d", alpha_slider_max );




 createTrackbar( TrackbarName_red, "threshold output for red", &alpha_slider_red, alpha_slider_max, on_trackbar_red );

 sprintf( TrackbarName_green, "Alpha green x %d", alpha_slider_max );


 createTrackbar( TrackbarName_green, "threshold output for green", &alpha_slider_green, alpha_slider_max, on_trackbar_green );

 sprintf( TrackbarName_blue, "Alpha blue x %d", alpha_slider_max );


 createTrackbar( TrackbarName_blue, "threshold output for blue", &alpha_slider_blue, alpha_slider_max, on_trackbar_blue );



merge(bgr,merged_image);

//imshow("merged_image", merged_image);

 /// Show some stuff
// on_trackbar( alpha_slider, 0 );

 /// Wait until user press some key
  
    }
    // the camera will be deinitialized automatically in VideoCapture destructor/
    return 0;
}
