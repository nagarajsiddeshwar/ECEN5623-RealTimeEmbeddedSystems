#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <X11/Xlib.h>
#include <sched.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv/cv.h>

#include <semaphore.h>
//CvCapture* capture;f

#include <string.h>
#include <opencv2/core/utility.hpp>
//#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>


#include <syslog.h>
#include <math.h>
#include <sys/param.h>
#include <sys/time.h>
#include <errno.h>

#define NUM_THREADS		2
//#define START_SERVICE 		0
#define THREAD_1	        0
#define THREAD_2          	1
//#define THREAD_3 	        2
#define NUM_MSGS 		3

#define NSEC_PER_SEC (1000000000)
#define ERROR (-1)
#define OK (0)
#define MAX_OBJECTS (5)

using namespace cv;
using namespace std;



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

Mat imgOriginal,imgHSV,imgThresholded,drawing;

 //cv::VideoWriter output_cap("/home/nagarajcs/Desktop/RTES.avi",CV_FOURCC('M','J','P','G'), 1, cv::Size ( 640,480), true);

pthread_t threads[NUM_THREADS];

struct sched_param nrt_param;

pthread_attr_t rt_sched_attr[NUM_THREADS];
struct sched_param rt_param[NUM_THREADS];

pthread_mutex_t lock1;
pthread_mutexattr_t rt_safe;

sem_t frame,centroid;

VideoCapture cap(0);

/*
void *sequencer(void *arg)//sequence_t will point to this
{

	struct timespec tv1;
	struct timespec tv2;
	tv1.tv_sec=0;
	tv1.tv_nsec=50000000; // 20 Hz


	int count=0;
	while(1)
	{	
		if(count%2==0)   // 10 Hz
		{
                sem_post(&frame);
                }
		
		if(count%5==0)  // 2 Hz
		{
		sem_post(&centroid);
		}
		

		count++;
		nanosleep(&tv1,&tv2);
	}

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
 namedWindow("Original", CV_WINDOW_AUTOSIZE);
  //capture the video from web cam


    if ( !cap.isOpened() )  // if not success, exit program
    {
         
         cout << "Cannot open the web cam" << endl;
        
        // return -1;
    }
      while (true)
    {
		//sem_wait(&frame);
                start=readTOD();
        
		
		// Capturing the input frame from the connected camera device 

pthread_mutex_lock(&lock1);
     //   bool bSuccess = cap.read(imgOriginal); // reooad a new frame from video
         cap.read(imgOriginal);

        // imshow("Original",imgOriginal); 
      //   waitKey(1);
pthread_mutex_unlock(&lock1); 

                         stop=readTOD();
                        syslog(LOG_INFO, "FrameCapture thread difference time msec= %lf \n", (double)(stop-start));
		

    }
      pthread_exit(NULL);

}

void *CentroidDetection(void * threadid)  //int main( int argc, char** argv ) 
{
 
syslog(LOG_INFO, "%s", "Entered Centroid Detection frame \n");

	
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
       cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 cvCreateTrackbar("HighH", "Control", &iHighH, 179);

 cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 cvCreateTrackbar("HighS", "Control", &iHighS, 255);

 cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 cvCreateTrackbar("HighV", "Control", &iHighV, 255);
  while (true)
    {
	//sem_wait(&centroid);
        start=readTOD();
       pthread_mutex_lock(&lock1); 
       medianBlur(imgOriginal, imgOriginal, 3);
      medianBlur(imgOriginal, imgOriginal, 3);
       cvtColor(imgOriginal, imgHSV, CV_RGB2HSV);
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

	//	imshow("Thresholded Image", imgThresholded); //show the thresholded image
                
		//imshow("Original", imgOriginal); //show the original image
               printf("\n rectlength = %d , rectheight = %d \n", rectlength, rectheight); 
          //     if((rectlength*rectheight)>(640*480*0.1))

		//imshow( "Contours",drawing );

              //   imshow( "Contours", drawing );

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
		    cout << "esc key is pressed by user" << endl;
		    break; 
		}

           //   output_cap.write( drawing);
  

 
          pthread_mutex_unlock(&lock1); 


                         stop=readTOD();
                        syslog(LOG_INFO, "Centroid Detection thread difference time msec= %lf \n", (double)(stop-start));
}


 pthread_exit(NULL);
}


int main( int argc, char** argv )
{


XInitThreads();

    // print_scheduler();

  
/*
	sem_init(&frame, 0, 0);
        sem_init(&centroid, 0, 0);
*/	
	pthread_mutex_init(&lock1, NULL);
     


     pthread_attr_init(&rt_sched_attr[THREAD_1]);
   pthread_attr_setinheritsched(&rt_sched_attr[THREAD_1], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[THREAD_1], SCHED_FIFO);

   pthread_attr_init(&rt_sched_attr[THREAD_2]);
   pthread_attr_setinheritsched(&rt_sched_attr[THREAD_2], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[THREAD_2], SCHED_FIFO);

   rt_param[THREAD_1].sched_priority = 1;
   pthread_attr_setschedparam(&rt_sched_attr[THREAD_1], &rt_param[THREAD_1]);

      rt_param[THREAD_2].sched_priority = 2;
   pthread_attr_setschedparam(&rt_sched_attr[THREAD_2], &rt_param[THREAD_2]);

   printf("Creating thread %d\n", THREAD_1);
   int rc = pthread_create(&threads[0], &rt_sched_attr[THREAD_1], FrameCapture, (void *)0);
   if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
   printf("Thread 1 spawned\n");

   printf("Creating thread %d\n", THREAD_2);
   rc = pthread_create(&threads[1], &rt_sched_attr[THREAD_2], CentroidDetection, (void *)0);
   if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
   printf("Thread 2 spawned\n");


    pthread_join(threads[0], NULL);
    pthread_join(threads[1], NULL);

   printf("All done\n");

   exit(0);
    
};
