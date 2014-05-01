// this code is based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/PointStamped.h>

namespace enc = sensor_msgs::image_encodings;

cv::Mat GetThresholdedImage(cv::Mat imgHSV);
void trackObject(cv::Mat imgThresh);
double posX=0;
double posY=0;

class ImageTracker
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher pub;
public:
  ImageTracker()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("image_tracked", 1);
    image_sub_ = it_.subscribe("image", 1, &ImageTracker::imageCb, this);
    pub = nh_.advertise<geometry_msgs::PointStamped>("/image_tracked/trackedpoint",10);

    cv::namedWindow("COLOR_TRACKING");
    cv::namedWindow("BINARY");
  }

  ~ImageTracker()
  {
    cv::destroyWindow("COLOR_TRACKING");
    cv::destroyWindow("BINARY");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }
    //start the tracking process
    cv::Mat frame;
    frame = cv_ptr->image.clone();
    //smooth image
    cv::GaussianBlur(frame, frame, cv::Size(3,3), 0, 0);
    cv::Mat imgHSV;
    cv::cvtColor(frame, imgHSV, CV_BGR2HSV);
    cv::Mat imgThresh = GetThresholdedImage(imgHSV);
    trackObject(imgThresh);
    if(posX>=0 && posY>=0){
      std::cout<<"Object located! "<<"X="<<posX<<" Y="<<posY<<std::endl;
      cv::circle(cv_ptr->image, cv::Point(posX,posY),3,cv::Scalar(0,0,255),-1);
      cv::circle(imgThresh, cv::Point(posX,posY),8,cv::Scalar(0,0,255),-1);
      cv::rectangle(cv_ptr->image,cv::Point(posX-10,posY-10),cv::Point(posX+10,posY+10),cv::Scalar(255,255,255));
    
       //publish the tracked location
       geometry_msgs::PointStamped msg;
       msg.header.stamp = ros::Time().now();
       msg.point.x = posX;
       msg.point.y = posY;
       ROS_INFO("published x=%5.1f y=%5.1f\n",msg.point.x,msg.point.y);
       pub.publish(msg);

    }
    

    cv::imshow("BINARY",imgThresh);
    cv::imshow("COLOR_TRACKING", cv_ptr->image);
    cv::waitKey(100);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_tracker");
  ImageTracker ic;

  ros::spin();
    return 0;
}


cv::Mat GetThresholdedImage(cv::Mat imgHSV){
  cv::Mat imgThresh = cvCreateMat(imgHSV.size().height,imgHSV.size().width,CV_8UC1);
  //blue
  cv::inRange(imgHSV,cv::Scalar(100,150,50),cv::Scalar(140,256,200),imgThresh);
  //red
  //cv::inRange(imgHSV,cv::Scalar(0,110,50),cv::Scalar(35,256,256),imgThresh);

  return imgThresh;
}
  

void trackObject(cv::Mat imgThresh){
  cv::Moments m = cv::moments(imgThresh);
  if(m.m00>2000){
  posX = m.m10/m.m00;
  posY = m.m01/m.m00;
  }


}
