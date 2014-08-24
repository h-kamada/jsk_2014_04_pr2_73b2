#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <face_detector_mono/RectArray.h>
#include <face_detector_mono/Rect.h>

using namespace std;
using namespace ros;
using namespace cv;
#define SIZE_X=480 // must be modified to use camera_info
#define SIZE_Y=640 

typedef struct TP{
  TP(){
  }
  TP(cv::Mat temp, cv::Ptr<cv::FeatureDetector> detector,cv::Ptr<cv::DescriptorExtractor> extractor){
    img = temp;
    detector->detect(img, keypoint);
    extractor->compute(img, keypoint, descriptor);
  }
  cv::Mat img;
  std::vector<cv::KeyPoint> keypoint;
  cv::Mat descriptor;
} template_data;

class Learner_Class{
public:
  ros::NodeHandle n;
  ros::Publisher point_pub;
  ros::Subscriber sub_regist;
  image_transport::ImageTransport _it;
  image_transport::Subscriber sub_img;
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat tmp_img;
  std::vector<template_data> template_imgs;
  std::vector<cv::KeyPoint> keypoint_img;
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Mat descriptor_img;
  cv::Ptr<cv::DescriptorMatcher> matcher;
  bool start_flag;
  Learner_Class(): _it(n), detector(cv::FeatureDetector::create("SIFT")), extractor(cv::DescriptorExtractor::create("SIFT")), matcher(cv::DescriptorMatcher::create("BruteForce")){
    start_flag = false;
    sub_img = _it.subscribe("/image_raw", 10, &Learner_Class::image_cb, this);
    sub_regist = n.subscribe("/regist_command", 10, &Learner_Class::regist_cb, this);
    point_pub = n.advertise<face_detector_mono::RectArray>("/nao_learn", 10);
    ROS_INFO("start subscribing");
    cv::initModule_nonfree();
  }
  void image_cb(const sensor_msgs::ImageConstPtr& msg_ptr){
    try{
      cv_ptr = cv_bridge::toCvCopy(msg_ptr, "bgr8");
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    tmp_img = cv_ptr->image.clone();
    cv::Mat red_img = tmp_img.clone();
    cv::rectangle(red_img,cv::Point(160, 120), cv::Point(480, 360), cv::Scalar(0, 0, 200), 3, 4);
    cv::imshow("result", red_img);
    int count=0;
    detector->detect(tmp_img, keypoint_img);
    extractor->compute(tmp_img, keypoint_img, descriptor_img);
    face_detector_mono::RectArray msg;
    for (std::vector<template_data>::iterator pt = template_imgs.begin(); pt != template_imgs.end(); pt++){
      count++;
      char window_name[16];
      sprintf(window_name, "template%d", count);
      cv::imshow(window_name, pt->img); 
      //check matching
      std::vector< DMatch > matches;
      matcher->match(pt->descriptor, descriptor_img, matches);
      double max_dist = 0; double min_dist = 100;

      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < pt->descriptor.rows; i++ )
      	{ double dist = matches[i].distance;
      	  if( dist < min_dist ) min_dist = dist;
      	  if( dist > max_dist ) max_dist = dist;
      	}

      printf("-- Max dist : %f \n", max_dist );
      printf("-- Min dist : %f \n", min_dist );

      //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
      std::vector< DMatch > good_matches;
      
      for( int i = 0; i < pt->descriptor.rows; i++ )
      	{ if( matches[i].distance < 3*min_dist )
      	    { good_matches.push_back( matches[i]); }
      	}
      
      Mat img_matches;
      drawMatches( pt->img, pt->keypoint, tmp_img, keypoint_img,
      		   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
      		   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

      std::vector<Point2f> obj;
      std::vector<Point2f> scene;
      int count_p;
      for(count_p = 0; count_p < good_matches.size(); count_p++ )
      	{
      	  //-- Get the keypoints from the good matches
      	  obj.push_back( pt->keypoint[ good_matches[count_p].queryIdx ].pt );
      	  scene.push_back( keypoint_img[ good_matches[count_p].trainIdx ].pt );
      	}
      if(good_matches.size() < 10){
	sprintf(window_name, "matches%d", count);
	cv::imshow(window_name, img_matches);
	continue;
      }
      Mat H = findHomography( obj, scene, CV_RANSAC );
      //-- Get the corners from the image_1 ( the object to be "detected" )
      std::vector<Point2f> obj_corners(4);
      obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( pt->img.cols, 0 );
      obj_corners[2] = cvPoint( pt->img.cols, pt->img.rows ); obj_corners[3] = cvPoint( 0, pt->img.rows );
      std::vector<Point2f> scene_corners(4);

      perspectiveTransform( obj_corners, scene_corners, H);

      //-- Draw lines between the corners (the mapped object in the scene - image_2 )
      line( img_matches, scene_corners[0] + Point2f( pt->img.cols, 0), scene_corners[1] + Point2f( pt->img.cols, 0), Scalar(0, 255, 0), 4 );
      line( img_matches, scene_corners[1] + Point2f( pt->img.cols, 0), scene_corners[2] + Point2f(pt->img.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[2] + Point2f( pt->img.cols, 0), scene_corners[3] + Point2f(pt->img.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[3] + Point2f( pt->img.cols, 0), scene_corners[0] + Point2f(pt->img.cols, 0), Scalar( 0, 255, 0), 4 );
      sprintf(window_name, "matches%d", count);
      cv::imshow(window_name, img_matches);
      face_detector_mono::Rect a;
      a.x = (scene_corners[1].x + scene_corners[3].x)/2; 
      a.y = (scene_corners[1].y + scene_corners[3].y)/2; 
      a.height=480; a.width=640;
      msg.rects.push_back(a);
     }

    point_pub.publish(msg);
    cv::waitKey(20);
   
    start_flag=true;
  }

  void regist_cb(const std_msgs::String& st){ //will be changed to change size and position
    if(start_flag){
      cv::Mat cut_img(tmp_img, cv::Rect(160, 120, 320,240));

      //      template_imgs.push_back(template_data(cut_img, detector, extractor));
      
      cv::Mat dst_img;
      cv::resize(cut_img, dst_img, cv::Size(), 0.5, 0.5);
      template_imgs.push_back(template_data(dst_img, detector, extractor));

      ROS_INFO("regist was finished");
    }
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "object_learner");
  Learner_Class LC;
  ROS_INFO("start learning");
  ros::spin();
  return 0;
}
