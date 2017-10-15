#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if ((*cv_ptr).image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      
      
 
 
 int iLowH =0;
 int iHighH = 179;

 int iLowS = 0;
 int iHighS = 255;

 int iLowV = 0;
 int iHighV = 97;

 

 Mat image=cv_ptr->image;

      
 //Create trackbars in "OPENCV_WINDOW" window
 cvCreateTrackbar("LowH", "OPENCV_WINDOW", &iLowH, 179); //Hue (0 - 179)
 cvCreateTrackbar("HighH", "OPENCV_WINDOW", &iHighH, 179);

 cvCreateTrackbar("LowS", "OPENCV_WINDOW", &iLowS, 255); //Saturation (0 - 255)
 cvCreateTrackbar("HighS", "OPENCV_WINDOW", &iHighS, 255);

 cvCreateTrackbar("LowV", "OPENCV_WINDOW", &iLowV, 255); //Value (0 - 255)
 cvCreateTrackbar("HighV", "OPENCV_WINDOW", &iHighV, 255);

 Mat imgHSV;

  cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  Mat imgThresholded;

  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (remove small objects from the foreground);
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground);
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  imshow("Thresholded Image", imgThresholded); //show the thresholded image
 
  int i,j,fact1=64*2,fact=48*2,m,n;
  int intensity[5][5];
  int value[]={-2,-1,0,1,2};
  long intensitypriority=0;
  long intensitysum=0;
  int pos=0;
 for(i=0;i<5;i++){
	  for(j=0;j<5;j++){
		    intensity[i][j]=0;
		 int count = 0;
		  for(m=i*fact;m<i*fact+fact;m++){
			  for(n=j*fact1;n<j*fact1+fact1;n++){
				  
				        
						
						Mat HSV;
						//Mat RGB=image(Rect(n,m,1,1));
						//cvtColor(RGB, HSV,CV_BGR2HSV);
						Vec3b hsv=imgHSV.at<Vec3b>(m,n);
						int H= hsv.val[0];
						int S= hsv.val[1];
						int V=hsv.val[2];
						if(H>0 && H<179 && S>0 && S<255 && V>0 && V<97){
							count++;
						}
						if(H>=iLowH && H<=iHighH && S>=iLowS && S<=iHighS && V>=iLowV && V<=iHighV){
							count++;
						}
					
			  }
		  }
		  if(count >300){
			intensity[i][j]=count/10;
		  }
		 
		  
 
intensitypriority += intensity[i][j]* value[j]*(i+1)*10;//lculating the weighted mean 
//printf("%d		",intensity[i][j]* value[j]);
//cout<<count<<"		";    
intensitysum +=intensity[i][j];        //Calculating sum of sensor readings 
		  }
	  //printf("\n");
  }
   if(intensitysum!=0){
		pos=int(intensitypriority/intensitysum);
   }
 
 printf("position is %d\n",pos);
			  
  
  for(i=0;i<=5;i++){
  line( image, Point(i*fact1,0), Point(i*fact1,480 ), Scalar( 0, 0, 0 ),2,8 );
  }
  for(j=0;j<=5;j++){
	line( image, Point(0,j*fact), Point( 640,j*fact ), Scalar( 0, 0, 0 ),2,8);
   }
 
  imshow("Original", image); //show the original image
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
