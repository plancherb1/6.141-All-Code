#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "ros/ros.h" // main ROS include
#include "std_msgs/Float32.h" // number message datatype
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

// simple class to contain the node's variables and code
class IBVSNode
{
  public:
    IBVSNode(); // constructor

  private:
    ros::NodeHandle nh_; // interface to this node
    image_transport::ImageTransport self_;
    image_transport::Subscriber IBVS_sub_;
    ros::Publisher IBVS_pub_;

    // callback function declarations
    void centerPointCallback(const sensor_msgs::ImageConstPtr& msg);
};

// program entry point
int main(int argc, char *argv[])
{
  // initialize the ROS client API, giving the default node name
  ros::init(argc, argv, "IBVS_node");

  IBVSNode node;

  // enter the ROS main loop
  ros::spin();

  return 0;
}



IBVSNode::IBVSNode() :
  self_(nh_)
{
  // subscribe to the zed camera topic and call centerPointCallback
  IBVS_sub_ = self_.subscribe("zed_camera", 1, &IBVSNode::centerPointCallback, this);
  // advertise that we'll publish on the IBVS_pub
  IBVS_pub_ = nh_.advertise <ackermann_msgs::AckermannDriveStamped> ("vesc/ackermann_cmd_mux/input/navigation", 1);
}

// the callback function for the number stream topic subscription
void IBVSNode::centerPointCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cout << " Success" << endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    namedWindow("Control", CV_WINDOW_NORMAL); //create a window called "Control"
    resizeWindow("Control", 320, 240);

    //adjusted for image color
    int iLowH = 0;
    int iHighH = 22;
    int iLowS = 196;
    int iHighS = 255;
    int iLowV = 63;
    int iHighV = 255;
    
  //Capture a temporary image from the camera
  Mat imgTmp;
  imgTmp = cv_ptr->image;
  //Create a black image with the size as the camera output
  Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;

      Mat imgOriginal;
      imgOriginal = cv_ptr->image;
      
      int iAngle = 90;
      //get the affine transformation matrix
      //Mat matRotation = getRotationMatrix2D( Point(imgOriginal.cols / 2, imgOriginal.rows / 2), (iAngle - 180), 1 );

      // Rotate the image
      //Mat imgRotated;
     // warpAffine( imgOriginal, imgOriginal, matRotation, imgOriginal.size() );

      Mat imgHSV;
      cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
      Mat imgThresholded;
      inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

      //morphological opening (removes small objects from the foreground)
      erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

      //morphological closing (removes small holes from the foreground)
      dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

      //Calculate the moments of the thresholded image
      Moments oMoments = moments(imgThresholded);

      double dM01 = oMoments.m01;
      double dM10 = oMoments.m10;
      double dArea = oMoments.m00;

      // if the area <= 5000, cone too far away
      if (dArea > 5000)
      {
        //calculate the position of the ball
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;
			cout << " X" << posX << " Y " << posY << endl;

			const int MID_POINT = 640;
			const int buffer = 30;
			int deltaX = posX-MID_POINT;
			cout << "deltaX " << deltaX << endl;
			if (deltaX>=buffer || deltaX<=-1*buffer)
			{
				ackermann_msgs::AckermannDriveStamped turn_msg; 
				if   (deltaX < -1*buffer) 
				//publish turn left			
				{
				cout << "turn left!" << endl;
				turn_msg.drive.steering_angle = -1.0*deltaX/MID_POINT;
				cout << turn_msg.drive.steering_angle << endl;
				turn_msg.drive.speed = 0.0;
				IBVS_pub_.publish(turn_msg);
				}
				else if (deltaX > buffer)
				{
				//publish turn right
				turn_msg.drive.steering_angle = -1.0*deltaX/MID_POINT;
				cout << turn_msg.drive.steering_angle << endl;
				turn_msg.drive.speed = 0.0;
				cout << "turn right!" << endl;}
				IBVS_pub_.publish(turn_msg);
				}
		}
/*
	//Draw bounding rectangle and contours and centroid point
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	circle(imgOriginal, Point(posX, posY), 5, Scalar(255,255,255), 5);
	findContours( imgThresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
	
	vector<RotatedRect> minRect( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
		minRect[i] = minAreaRect( Mat(contours[i]) );

	Mat drawing = Mat::zeros( imgThresholded.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
		drawContours( drawing, contours, i, Scalar(0,255,255), 10, 8, vector<Vec4i>(), 0, Point() );
		// rotated rectangle
		Point2f rect_points[4]; minRect[i].points( rect_points );
		//	for( int j = 0; j < 4; j++ )
		//	line( drawing, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,255), 10, 8 );
	}
	imgOriginal = imgOriginal + drawing;
       }

	 namedWindow("Thresholded Image", CV_WINDOW_NORMAL); //create a window called "Threshold Image"
	resizeWindow("Thresholded Image", 320, 240);
	imshow("Thresholded Image", imgThresholded); //show the thresholded image

	namedWindow("Original", CV_WINDOW_NORMAL); //create a window called "Original"
	resizeWindow("Original", 320, 240);
	imshow("Original", imgOriginal); //show the original image
*/


      //need to calculate changes to cmd_vel based on posX and posY
      //find height and width (for frame ==1)
      //diff(posX, width) -> steering angle (need algorithm), until diff = 0


}
