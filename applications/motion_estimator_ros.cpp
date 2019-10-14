///ROS
#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
///Opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
////our include
#include <geometry.h>
#include <rgbd_loader.h>
#include <klt_tracker.h>
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>

using namespace std;
using namespace cv;

///rgbd_rtk variables
KLTTracker tracker;
Intrinsics intr(0);
MotionEstimatorRANSAC motion_estimator(intr);
//others variables
//ReconstructionVisualizer visualizer;
Eigen::Affine3f pose = Eigen::Affine3f::Identity();
Eigen::Affine3f trans = Eigen::Affine3f::Identity();
pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr curr_cloud(new pcl::PointCloud<PointT>);
int i=0;


void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD); //subscribing rgb and depth image
void motionEstimator(cv::Mat rgb , cv::Mat depth); //motion estiomator


int main(int argc, char** argv){
  string filename = "../../../src/ros_autonomous_robot/ConfigFile.yaml";
  FileStorage fs(filename, FileStorage::READ);
  fs.open(filename, FileStorage::READ);

  ros::init(argc, argv, "bag_loader"); //initializing ros
  ros::start();
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, fs["rgb_topic"], 1); //subscribing to rgb topic
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, fs["depth_topic"], 1);  //subscribing to depth topic
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

  motionEstimator(cv_ptrRGB->image, cv_ptrD->image);
}
void motionEstimator(cv::Mat rgb , cv::Mat depth){

  *curr_cloud = getPointCloud(rgb, depth, intr);

  //track feature points in current frame
  tracker.track(rgb);
  cout<<i<<endl;

  if(i > 0){     //Estimate motion between the current and the previous frame/point clouds
    trans = motion_estimator.estimate(tracker.prev_pts_, prev_cloud, tracker.curr_pts_, curr_cloud);
    pose = pose*trans;
  }

  for(size_t k=0; k < tracker.curr_pts_.size(); k++){     //View tracked points
    cv::Point2i pt1 = tracker.prev_pts_[k];
    cv::Point2i pt2 = tracker.curr_pts_[k];
    cv::circle(rgb, pt1, 1, CV_RGB(255,0,0), -1);
    cv::circle(rgb, pt2, 3, CV_RGB(0,0,255), -1);
    cv::line(rgb, pt1, pt2, CV_RGB(0,0,255));
  }
    
  //3D vizualization 
  /*
  if(i == 0) visualizer.addReferenceFrame(pose, "origin");
  visualizer.addQuantizedPointCloud(curr_cloud, 0.3, pose);
  visualizer.viewReferenceFrame(pose);
  visualizer.viewPointCloud(curr_cloud, pose);
  visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, pose);
  //Spin Once = waitkey for 3D window
  visualizer.spinOnce();
  */
    
  depth= depth/5;
  cv::imshow("OPENCV_WINDOW_RGB", rgb);
  cv::imshow("OPENCV_WINDOW_DEPTH", depth);
  cv::waitKey(1);
  //Let the prev. cloud in the next frame be the current cloud
  *prev_cloud = *curr_cloud;
  i++;
}


