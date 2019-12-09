#include <cstdio>
#include <cstdlib>
///ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
///Opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
////our include
#include <config_loader.h>
#include <geometry.h>
#include <rgbd_loader.h>
#include <klt_tracker.h>
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>

using namespace std;
using namespace cv;

///rgbd_rtk variables
KLTTracker tracker;
ConfigLoader Loader;
Intrinsics intr(0);
MotionEstimatorRANSAC motion_estimator(intr);
//others variables
ReconstructionVisualizer visualizer;
Eigen::Affine3f pose = Eigen::Affine3f::Identity();
Eigen::Affine3f trans = Eigen::Affine3f::Identity();
pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr curr_cloud(new pcl::PointCloud<PointT>);
string rgb_topic, depth_topic;
int i=0;


void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD); //subscribing rgb and depth image
void motionEstimator(cv::Mat rgb , cv::Mat depth); //motion estiomator
void paramLoader();
void minMaxDebug(Mat depth, Mat rgb, double &min, double &max);

int main(int argc, char** argv){
  //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
  Loader.loadParams("../../../src/ros_autonomous_robot/ConfigFile.yaml"); //Load config file param
  
  ros::init(argc, argv, "motion_estimator"); //initializing ros
  ros::start();
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, Loader.rgb_topic_, 1); //subscribing to rgb topic
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, Loader.depth_topic_, 1);  //subscribing to depth topic
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy; //defining which topics will be sync
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), rgb_sub,depth_sub); //sync rgb and depth topic in "MySyncPolicy(x)" miliseconds
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
/**
 * Ros Callback, listen to rgb and depth topic
 * @Params rgb and depth messages 
 */
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
  //convert the depth image CV_32 to CV_16UC1
  Mat  depth_unsigned_short;
  cv_ptrD->image.convertTo(depth_unsigned_short, CV_16UC1, 1000, 0.0); 
  //Call motion estimator
  motionEstimator(cv_ptrRGB->image, depth_unsigned_short);
}
/**
 * Motion estimator using keypoints
 * @Params rgb and depth images as cv::Mat
 */
void motionEstimator(cv::Mat rgb , cv::Mat depth){

  *curr_cloud = getPointCloud(rgb, depth, intr);
  //track feature points in current frame
  tracker.track(rgb);

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
  if(i == 0) visualizer.addReferenceFrame(pose, "origin");
  visualizer.addQuantizedPointCloud(curr_cloud, 0.3, pose);
  visualizer.viewReferenceFrame(pose);
  visualizer.viewPointCloud(curr_cloud, pose);
  //visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, pose);
  visualizer.spinOnce();
    
  depth /= 5;
  cv::imshow("OPENCV_WINDOW_RGB", rgb);
  cv::imshow("OPENCV_WINDOW_DEPTH", depth);
  cv::waitKey(1);
  //Let the prev. cloud in the next frame be the current cloud
  *prev_cloud = *curr_cloud;
  i++;
}

  /**
   * Get the min and max distance in a depth image
   * 
   * @Param Depth image and rgb image as cv::Mat; min(insert as "infinite") and max(insert as 0)  as reference
   * @Return min and max values as reference
   */
void minMaxDebug(Mat depth, Mat rgb, double &min, double &max){

  max=depth.at<unsigned short>(0,0);
  for(int i = 0; i<depth.rows; i++){
    for(int j = 0; j<depth.cols; j++){
      //if(depth.at<unsigned short>(i,j) >=25000 ){
      //  cv::circle(rgb, Point(j,i), 1, CV_RGB(255,0,0), -1);
      //}
      if(depth.at<unsigned short>(i,j) != depth.at<unsigned short>(i,j)){
        continue;
      }
      if(depth.at<unsigned short>(i,j) == 0.0){
        continue;
      }
      if(depth.at<unsigned short>(i,j) < min){
        min = depth.at<unsigned short>(i,j);
      }

      if(depth.at<unsigned short>(i,j) > max){
        max = depth.at<unsigned short>(i,j);
      }

    }
  }

}
