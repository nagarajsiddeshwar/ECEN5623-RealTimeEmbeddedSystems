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
//CvCapture* capture;f

#include <string.h>

#include <syslog.h>
#include <math.h>
#include <sys/param.h>
#include <sys/time.h>
#include <errno.h>

#define NUM_THREADS		3
//#define START_SERVICE 		0
#define THREAD_1	        0
#define THREAD_2          	1
#define THREAD_3 	        2
#define NUM_MSGS 		3

#define NSEC_PER_SEC (1000000000)
#define ERROR (-1)
#define OK (0)
#define MAX_OBJECTS (5)

using namespace cv;
using namespace std;

static struct timespec rtclk_frameCapture_start_time = {0, 0};
static struct timespec rtclk_frameCapture_stop_time = {0, 0};
static struct timespec rtclk_frameCapture_difference = {0,0};
static struct timespec rtclk_centroidDetection_start_time = {0, 0};
static struct timespec rtclk_centroidDetection_stop_time = {0, 0};
static struct timespec rtclk_centroidDetection_difference = {0,0};
static struct timespec rtclk_obstacleMovement_start_time = {0, 0};
static struct timespec rtclk_obstacleMovement_stop_time = {0, 0};
static struct timespec rtclk_obstacleMovement_difference = {0,0};
static struct timespec rtclk_videoOuput_start_time = {0, 0};
static struct timespec rtclk_videoOuput_stop_time = {0, 0};
static struct timespec rtclk_videoOuput_difference = {0,0};



double start=0,stop=0;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
    int ilastx=-1,ilasty=-1; 
    //int iLastY = -1;
vector<int> tracex,tracey;
RNG rng(12345);
Mat retrieve_element; 
int videocapture_count = 0;
static int video_count = 0;
int rectlength, rectheight;
 int iLowH = 0;
 int iHighH = 179;

 int iLowS = 0; 
 int iHighS = 255;

 int iLowV = 0;
 int iHighV = 255;

Mat imgOriginal;

 //cv::VideoWriter output_cap("/home/nagarajcs/Desktop/RTES.avi",CV_FOURCC('M','J','P','G'), 1, cv::Size ( 640,480), true);

pthread_t threads[NUM_THREADS];

struct sched_param nrt_param;

pthread_attr_t rt_sched_attr[NUM_THREADS];
struct sched_param rt_param[NUM_THREADS];

pthread_mutex_t frame,centroid,store;
pthread_mutexattr_t rt_safe;

static sem_t sem_THREAD;
 Mat imgHSV,drawing;
Mat imgThresholded;

VideoCapture cap(0);
/*
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  //Calculating the time difference 
  if(dt_sec >= 0)
  {
	if(dt_nsec >= 0)
	{
	  delta_t->tv_sec=dt_sec;
	  delta_t->tv_nsec=dt_nsec;
	}
	else
	{
	  delta_t->tv_sec=dt_sec-1;
	  delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
	}
  }
  else
  {
	if(dt_nsec >= 0)
	{
	  delta_t->tv_sec=dt_sec;
	  delta_t->tv_nsec=dt_nsec;
	}
	else
	{
	  delta_t->tv_sec=dt_sec-1;
	  delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
	}
  }
  return(OK);
}

*/
double readTOD(void)
{
   struct timeval tv;
 double ft=0.0;

if(gettimeofday(&tv,NULL)!=0)
{
  perror("readTOD");
  return 0.0;
}
else 
{
ft= ((double)(((double)tv.tv_sec)+(((double)tv.tv_usec)/1000000.0)));
}
return ft;
}
void print_scheduler(void)
{
	int schedType;
	schedType = sched_getscheduler(getpid());
	switch(schedType)
	{
		case SCHED_FIFO:
			syslog(LOG_INFO, "%s", "Pthread Policy is SCHED_FIFO\n");
		break;

		case SCHED_OTHER:
			syslog(LOG_INFO, "%s", "Pthread Policy is SCHED_OTHER\n");
		break;

		case SCHED_RR:
			syslog(LOG_INFO, "%s", "Pthread Policy is SCHED_RR\n");
		break;

		default:
			syslog(LOG_INFO, "%s", "Pthread Policy is UNKNOWN\n");
		break;
	}
}

void *FrameCapture( void *threadid )

{
 
  //capture the video from web cam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         
         cout << "Cannot open the web cam" << endl;
        
        // return -1;
    }
      while (true)
    {
                start=readTOD();
        
/*
			clock_gettime(CLOCK_REALTIME, &rtclk_frameCapture_start_time);
			syslog(LOG_INFO, "FrameCapture thread starts time :: sec= %ld :: nsec= %ld \n",rtclk_frameCapture_start_time.tv_sec, rtclk_frameCapture_start_time.tv_nsec);
*/	
	
		
		/* Capturing the input frame from the connected camera device */

pthread_mutex_lock(&frame);
     //   bool bSuccess = cap.read(imgOriginal); // reooad a new frame from video
         cap.read(imgOriginal);
pthread_mutex_unlock(&frame); 
   /*      if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
           break;
        }
	*/	
/*	
			clock_gettime(CLOCK_REALTIME, &rtclk_frameCapture_stop_time);
			syslog(LOG_INFO, "FrameCapture thread stop time :: sec= %ld :: nsec= %ld \n",rtclk_frameCapture_stop_time.tv_sec, rtclk_frameCapture_stop_time.tv_nsec);
			delta_t(&rtclk_frameCapture_stop_time, &rtclk_frameCapture_start_time, &rtclk_frameCapture_difference);
			syslog(LOG_INFO, "FrameCapture thread difference time :: sec= %ld :: nsec= %ld \n", rtclk_frameCapture_difference.tv_sec, rtclk_frameCapture_difference.tv_nsec);
			exit(0);		
*/

                         stop=readTOD();
                        syslog(LOG_INFO, "FrameCapture thread difference time msec= %lf \n", (double)(stop-start));
		

    }
      pthread_exit(NULL);


}

    void *CentroidDetection(void * threadid)  //int main( int argc, char** argv ) 
{
syslog(LOG_INFO, "%s", "Entered Centroid Detection frame \n");
	
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"





 //Create trackbars in "Control" window
 cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 cvCreateTrackbar("HighH", "Control", &iHighH, 179);

 cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 cvCreateTrackbar("HighS", "Control", &iHighS, 255);

 cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  //cv::VideoWriter output_cap("/home/nagarajcs/Desktop/RTES.avi",CV_FOURCC('M','J','P','G'), 1, cv::Size ( 640,480), true);

    while (true)
    {
        start=readTOD();
       pthread_mutex_lock(&centroid); 

        
       medianBlur(imgOriginal, imgOriginal, 3);
       medianBlur(imgOriginal, imgOriginal, 3);
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        blur( imgHSV,imgHSV, Size(3,3) );
 
	

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (fill small holes in the foreground)
	
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	findContours( imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );


  for( int i = 0; i < contours.size(); i++ )
     { 
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );
		//    minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
     }

	        drawing = Mat::zeros(  imgThresholded.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ )

      {

		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                rectlength = (boundRect[i].br().x-boundRect[i].tl().x);

                rectheight=(boundRect[i].br().y-boundRect[i].tl().y);

		//  drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
             if((rectlength*rectheight)>(640*480*0.1*0))
           {
		rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
		Point center_of_rect = (boundRect[i].br() + boundRect[i].tl())*0.5;
		circle(drawing ,center_of_rect,3,Scalar(255,255,255));

		// iLast.x = boundRect[i].br();
		//             iLast.y = boundRect[i].tl();

		//printf("\n %d %d \n",boundRect[i].br().x*0.5+boundRect[i].tl().x*0.5,boundRect[i].br().y*0.5+boundRect[i].tl().y*0.5);
         
		if (ilastx >= 0 && ilasty >= 0 && (boundRect[i].br().y*0.5+boundRect[i].tl().y*0.5) >= 0 && (boundRect[i].br().x*0.5+boundRect[i].tl().x*0.5) >= 0)
	      {
			//Draw a red line from the previous point to the current point

	
		  // for(int count =0;count<100;count++)

			line(drawing, Point(boundRect[i].br().x*0.5+boundRect[i].tl().x*0.5, boundRect[i].br().y*0.5+boundRect[i].tl().y*0.5), Point(ilastx, ilasty), Scalar(0,0,255), 2);
		
	      }
               
		ilastx= boundRect[i].br().x*0.5+boundRect[i].tl().x*0.5;
                
		ilasty= boundRect[i].br().y*0.5+boundRect[i].tl().y*0.5;
                
	    }
       }

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
                
		imshow("Original", imgOriginal); //show the original image
               printf("\n rectlength = %d , rectheight = %d \n", rectlength, rectheight); 
          //     if((rectlength*rectheight)>(640*480*0.1))

		imshow( "Contours",drawing );

              //   imshow( "Contours", drawing );

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
		    cout << "esc key is pressed by user" << endl;
		    break; 
		}

           //   output_cap.write( drawing);
              
          pthread_mutex_unlock(&centroid); 


                         stop=readTOD();
                        syslog(LOG_INFO, "FrameCapture thread difference time msec= %lf \n", (double)(stop-start));
  
}
 pthread_exit(NULL);
}

void *StoreTracking( void *threadid )
{

cv::VideoWriter output_cap("/home/nagarajcs/Desktop/RTES.avi",CV_FOURCC('M','J','P','G'), 1, cv::Size ( 640,480), true);
    while(1)
    {
       
pthread_mutex_lock(&store); 
output_cap.write( drawing);
pthread_mutex_lock(&store); 

    }
    
 pthread_exit(NULL);

}

int main( int argc, char** argv )
{


XInitThreads();

    // print_scheduler();

    XInitThreads();
    pthread_mutex_init(&frame, NULL);
    
    pthread_mutex_init(&centroid, NULL);
 
    pthread_mutex_init(&store, NULL);
    

cvNamedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE);

cvNamedWindow("Original", CV_WINDOW_AUTOSIZE);

cvNamedWindow("Contours", CV_WINDOW_AUTOSIZE);

cvNamedWindow("Original", CV_WINDOW_AUTOSIZE);

     pthread_attr_init(&rt_sched_attr[THREAD_1]);
   pthread_attr_setinheritsched(&rt_sched_attr[THREAD_1], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[THREAD_1], SCHED_FIFO);

   pthread_attr_init(&rt_sched_attr[THREAD_2]);
   pthread_attr_setinheritsched(&rt_sched_attr[THREAD_2], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[THREAD_2], SCHED_FIFO);

   pthread_attr_init(&rt_sched_attr[THREAD_3]);
   pthread_attr_setinheritsched(&rt_sched_attr[THREAD_3], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[THREAD_3], SCHED_FIFO);

   rt_param[THREAD_1].sched_priority = 1;
   pthread_attr_setschedparam(&rt_sched_attr[THREAD_1], &rt_param[THREAD_1]);

      rt_param[THREAD_2].sched_priority = 2;
   pthread_attr_setschedparam(&rt_sched_attr[THREAD_2], &rt_param[THREAD_2]);

   rt_param[THREAD_3].sched_priority = 3;
   pthread_attr_setschedparam(&rt_sched_attr[THREAD_3], &rt_param[THREAD_3]);

   printf("Creating thread %d\n", THREAD_1);
   int rc = pthread_create(&threads[0], &rt_sched_attr[THREAD_1], FrameCapture, (void *)0);
   if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
   printf("Thread 1 spawned\n");

   printf("Creating thread %d\n", THREAD_2);
   rc = pthread_create(&threads[1], &rt_sched_attr[THREAD_2], CentroidDetection, (void *)0);
   if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
   printf("Thread 2 spawned\n");

      printf("Creating thread %d\n", THREAD_3);
   rc = pthread_create(&threads[2], &rt_sched_attr[THREAD_3], StoreTracking, (void *)0);
   if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
   printf("Thread 3 spawned\n");


    pthread_join(threads[0], NULL);
    pthread_join(threads[1], NULL);
    pthread_join(threads[2], NULL);
 

   printf("All done\n");

   exit(0);
    
};
