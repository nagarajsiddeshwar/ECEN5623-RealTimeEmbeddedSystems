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
using namespace std;

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
    int ilastx=-1,ilasty=-1; 
    //int iLastY = -1;

	int obstacle_yaxes = 480;
	int obstacle_xaxes;
	stringstream x_coordinate,y_coordinate, final_coordinates;
	Mat drawing=Mat::zeros( 480, 1281, CV_8UC3 );
	Point2i start_point(0,0),end_point(0,0);
	//Mat obstacle_window,trajectory_window;
	Mat obstacle_window = drawing(Rect(0,0,640,480));
	Mat trajectory_window = drawing(Rect(641,0,640,480));
	//line(drawing,Point(641,0),Point(641,480),CV_RGB(255,0,0),2);
	int trajectory_count =0;
	bool temp=false;
        int coordinate_x, coordinate_y;
        bool answer = false;
        int obstaclemovement_count = 0;
std::vector<int> tracex.tracey;

RNG rng(12345);

 int main( int argc, char** argv )
 {
    VideoCapture cap(0); //capture the video from web cam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

 int iLowH = 0;
 int iHighH = 179;

 int iLowS = 0; 
 int iHighS = 255;

 int iLowV = 0;
 int iHighV = 255;



 //Create trackbars in "Control" window
 cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 cvCreateTrackbar("HighH", "Control", &iHighH, 179);

 cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 cvCreateTrackbar("HighS", "Control", &iHighS, 255);

 cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

  Mat imgHSV;

  cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  blur( imgHSV,imgHSV, Size(3,3) );
 
  Mat imgThresholded;

  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (remove small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  findContours( imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
/*
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

  Mat drawing = Mat::zeros(  imgThresholded.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
 
    //  drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
   
   rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
   Point center_of_rect = (boundRect[i].br() + boundRect[i].tl())*0.5;
    circle(drawing ,center_of_rect,3,Scalar(255,255,255));

 // iLast.x = boundRect[i].br();
  //             iLast.y = boundRect[i].tl();

printf("\n %d %d \n",boundRect[i].br().x*0.5+boundRect[i].tl().x*0.5,boundRect[i].br().y*0.5+boundRect[i].tl().y*0.5);

            if (ilastx >= 0 && ilasty >= 0 && (boundRect[i].br().y*0.5+boundRect[i].tl().y*0.5) >= 0 && (boundRect[i].br().x*0.5+boundRect[i].tl().x*0.5) >= 0)
               {
                //Draw a red line from the previous point to the current point
                line(drawing, Point(boundRect[i].br().x*0.5+boundRect[i].tl().x*0.5, boundRect[i].br().y*0.5+boundRect[i].tl().y*0.5), Point(ilastx, ilasty), Scalar(0,0,255), 2);
               }
  ilastx= boundRect[i].br().x*0.5+boundRect[i].tl().x*0.5;
   ilasty= boundRect[i].br().y*0.5+boundRect[i].tl().y*0.5;

*/


obstacle_xaxes = coordinate_x;
		obstacle_yaxes = coordinate_y;
		temp = answer;
		//pthread_mutex_unlock(&rsrc_Coordinates);
		/* Copying the coordinate of the object for trajectory */
		if(trajectory_count == 0 && temp == true)
		/*Set start position for tracing the points on the screen*/
		{		
			start_point.x = obstacle_xaxes;		
			start_point.y = obstacle_yaxes;
			trajectory_count++;
		}
		else if(trajectory_count > 0 && temp == true)
		/*Keep adding new points to continue tracing the path*/
		{	
			end_point.x = obstacle_xaxes;		
			end_point.y = obstacle_yaxes;
			line(trajectory_window,start_point,end_point,CV_RGB(255,0,0),4,8); 
			start_point.x = end_point.x;
			start_point.y = end_point.y;	
		}
		else
		{
			
		}
		/* Check for corner case conditions */
		if(obstacle_xaxes < 20)
		{
			obstacle_xaxes = 20;
		}
		else if(obstacle_xaxes > 620)
		{
			obstacle_xaxes = 620;
		}
		else
		{
			
		}
		
		line(drawing,Point(obstacle_xaxes-20,480),Point(obstacle_xaxes+20,480),CV_RGB(255,0,0),12,8);  
               //line(drawing,Point(obstacle_xaxes-20,480),Point(obstacle_xaxes+20,480),CV_RGB(255,0,0),12,8); 
		(x_coordinate << obstacle_xaxes);
		(y_coordinate << 480);
		final_coordinates << "(" << x_coordinate.str() << "," << y_coordinate.str() << ")";
		//putText(drawing,Point((obstacle_xaxes),(480-10)),5,1,CV_RGB(255,0,0));
                putText(drawing,final_coordinates.str(),Point((obstacle_xaxes),(480-10)),5,1,CV_RGB(255,0,0));
		imshow( "Contours",drawing );
		char q = waitKey(2);
		if(q == 'q')
		{
			/* Clearing the trajectory window to draw new trajectory plot */
			trajectory_window.setTo(Scalar(0,0,0));
		}
		/* Clearing the obstacle window to draw new line */
		drawing.setTo(Scalar(0,0,0));
		final_coordinates.str(std::string());
		x_coordinate.str(std::string());
		y_coordinate.str(std::string());
		
		//#ifdef FREQUENCY_OBSTACLEMOVEMENT
	

  imshow("Thresholded Image", imgThresholded); //show the thresholded image
  waitKey(30);
  imshow("Original", imgOriginal); //show the original image
waitKey(30);
  //imshow( "Contours",drawing );
 



	obstaclemovement_count++;

}




   return 0;

}
