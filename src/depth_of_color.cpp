#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
double posX=0;
double posY=0;
double dis=0;

namespace enc = sensor_msgs::image_encodings;

class DepthCal
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher pub;
  ros::Subscriber sub;
public:
  DepthCal() 
    :it_(nh_)
  {
    image_pub_ = it_.advertise("pos_from_camera",1);
    image_sub_ = it_.subscribe("depth_image", 1, &DepthCal::depth_image_cb, this);
    
    pub = nh_.advertise<geometry_msgs::PointStamped>("pos_from_camera/points",10);
    sub = nh_.subscribe("image_tracked/trackedpoint", 1, &DepthCal::color_center_cb,this); 
    cv::namedWindow("Depth");
  }
  
  ~DepthCal()
  {
    cv::destroyWindow("Depth");
  }

  void color_center_cb(const geometry_msgs::PointStamped &msg)
  {
    //ROS_INFO("X: %lf Y:%lf",msg.point.x,msg.point.y);
    posX=msg.point.x;
    posY=msg.point.y;
  }

  void depth_image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }
    
    cv::Mat frame;
    frame = cv_ptr->image.clone();
    //ROS_INFO("type:%d rosws:%d cols:%d dims:%d channel:%d",frame.type(), frame.rows, frame.cols, frame.dims,frame.channels());
    
    
    int i, j, count = 0;
    double sum,avr = 0;
    for(i = (int)posX - 10;i <= (int)posX + 10;i++){
      for(j = (int)posY - 10;i <= (int)posY + 10;i++){
	if((!isnan(frame.at<float>(i,j))) && (fabs(frame.at<float>(i,j) - frame.at<float>(posX,posY)) < 0.3)){
	  sum+=frame.at<float>(i,j);
	  count ++;
	}
      }
    }
    avr = sum / count;
    
    ROS_INFO("value at [%lf %lf] =  %lf",posX, posY, frame.at<float>(posX,posY));
    ROS_INFO("average around=%lf", avr);
    
    cv::imshow("Depth",frame);
    cv::waitKey(100);
  }
};
  
    
  

int main(int argc, char** argv)
{
  ros::init(argc,argv,"pos_from_camera");
  DepthCal dc;

  ros::spin();
  return 0;
}
