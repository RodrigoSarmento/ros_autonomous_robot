
///ROS
#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
///Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


using namespace std;
using namespace cv;

void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

int main(int argc, char** argv){    

  string rgb_topic,depth_topic;
  rgb_topic = "camera/rgb/image_raw";
  depth_topic = "camera/depth/image_raw";
  if(argc != 1 && argc!= 3){
    fprintf(stderr, "Usage: %s optional: <rgb_topic> <depth_topic>....bye default : camera/rgb/image_raw and camera/depth/image_raw\n", argv[0]);
    exit(0);
  }
  if(argc == 1){
    printf(" By defult using camera/rgb/image_raw and camera/depth/image_raw as ros topics\n");  
  }
  if(argc == 3){
    rgb_topic = argv[1];
    depth_topic = argv[2]; 
  }
  

  ros::init(argc, argv, "bag_loader"); //initializing ros
  ros::start();
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 1); //subscribing to rgb topic
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 1);  //subscribing to depth topic
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy; //defining which topics will be sync
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), rgb_sub,depth_sub); //sync rgb and depth topic in "MySyncPolicy(x)" miliseconds
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
 }

void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD){

  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try{
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try{
    cv_ptrD = cv_bridge::toCvShare(msgD);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat depth = cv_ptrD->image;
  //showing rgb and depth images
  depth = depth/5;
  cv::imshow("OPENCV_WINDOW_RGB", cv_ptrRGB->image);
  cv::imshow("OPENCV_WINDOW_DEPTH", cv_ptrD->image);
  cv::waitKey(1);
}


