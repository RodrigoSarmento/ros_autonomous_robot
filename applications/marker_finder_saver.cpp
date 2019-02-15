#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>
///ROS
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include "nav_msgs/Odometry.h"
///Opencv
#include <opencv2/highgui/highgui.hpp>
//Aruco
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <marker_finder.h>

using namespace std;
using namespace cv;
using namespace aruco;

//where markers id and poses will be saved
struct markerFound{
  int id;
  float x_pose;
  float y_pose;
  float z_pose;
};
MarkerFinder marker_finder; //markerfinder
Eigen::Affine3f trans_camera_pose; //turtlebot pose
markerFound all_markers[255]; //marker struct
string all_markers_saved; //keyboard

void imageCallback(const sensor_msgs::ImageConstPtr& msg); //subscribe to rgb image
void rosMarkerFinder(cv::Mat rgb); //marker finder
void listenKeyboardSave(const std_msgs::String::ConstPtr& msg); //listening keyboard input for navigation
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); //subscribe turtlebot odometry
void initRos(int argc, char** argv,string rgb_topic);

int main(int argc, char** argv){  

  string rgb_topic;
  rgb_topic = "camera/rgb/image_raw";
  if(argc != 5 && argc !=4){
    fprintf(stderr, "Usage: %s <camera calibration file> <marker size> <aruco dic> <file where markes will be saved> optional: <rgb_topic> ....bye default : camera/rgb/image_raw \n", argv[0]);
    exit(0);
  }
  if(argc == 5){
    printf(" By defult using camera/rgb/image_raw as ros topic\n");  
  }
   if(argc == 6){
     rgb_topic = argv[5];
  }
  float marker_size;
  marker_size = stof(argv[2]);
  all_markers_saved = argv[4];
  marker_finder.markerParam(argv[1] , marker_size, argv[3]);

  for(int k=0; k<=254; k++){ //initializing markers
    all_markers[k].id = 0;
  }

  initRos(argc,argv,rgb_topic);

  return 0;
 }

void imageCallback(const sensor_msgs::ImageConstPtr& msgRGB){
  
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try{
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  rosMarkerFinder(cv_ptrRGB->image); //calling marker finder funcition
}

void rosMarkerFinder(cv::Mat rgb){
    //for testes
  Eigen::Affine3f I;
  I.setIdentity(); 
  marker_finder.detectMarkers(rgb, trans_camera_pose);   //Detect and get pose of all aruco markers

  for (size_t j = 0; j < marker_finder.markers_.size(); j++){
    all_markers[marker_finder.markers_[j].id].id = marker_finder.markers_[j].id;     //save all markers in a vetor 
    all_markers[marker_finder.markers_[j].id].x_pose = marker_finder.marker_poses_[j](0,3); //save marker position 
    all_markers[marker_finder.markers_[j].id].y_pose = marker_finder.marker_poses_[j](1,3);
    all_markers[marker_finder.markers_[j].id].z_pose = marker_finder.marker_poses_[j](2,3);
    cout << all_markers[marker_finder.markers_[0].id].x_pose << " "  << all_markers[marker_finder.markers_[0].id].y_pose << " " <<  all_markers[marker_finder.markers_[0].id].z_pose<<endl;
    marker_finder.markers_[j].draw(rgb, Scalar(0,0,255), 1);   //drawing markers in rgb image
    CvDrawingUtils::draw3dAxis(rgb, marker_finder.markers_[j], marker_finder.camera_params_);
    stringstream ss;
    ss << "m" << marker_finder.markers_[j].id;
  }
   
  cv::imshow("OPENCV_WINDOW", rgb);  //showing rgb image
  cv::waitKey(1);

}

void listenKeyboardSave(const std_msgs::String::ConstPtr& msg){
  
  string listen = msg->data.c_str();
  int cont=0;
  if(listen.compare("s") == 0){  //validing if string msg wants to save markers
    ofstream arq;
    arq.open(all_markers_saved);
    for(int k=0; k<=254; k++){
      if(all_markers[k].id==0) continue;
        cont ++;
    }
    arq<<cont<<endl;
    for(int k=0; k<=254; k++){
      if(all_markers[k].id==0) continue;
        arq<<all_markers[k].id<<" "<<all_markers[k].x_pose<<" "<<all_markers[k].y_pose <<endl;   //saving all markers in "all_markers.txt"
    }
  }
  else 
    ROS_INFO("[%s] is not a valid input, use 's' to save all markers", msg->data.c_str());
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

  Eigen::Vector3f v_robot(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z); //subscribing turtlebot pose 
  Eigen::Quaternionf q_robot(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z); ///subscribing turtlebot orientation pose 
  

  Eigen::Matrix3f R_robot = q_robot.normalized().toRotationMatrix();   // convert a quaternion to a 3x3 rotation matrix:
  

  Eigen::Matrix4f robot_pose;
  robot_pose.setIdentity();   // Set to Identity 
  robot_pose.block<3,3>(0,0) = R_robot; 
  robot_pose.block<3,1>(0,3) = v_robot;

  Eigen::Vector3f v_camera(-0.0125, 0.287, 0.087); //subscribing turtlebot pose 
  Eigen::Quaternionf q_camera(0.5, -0.5, 0.5, -0.5); ///subscribing turtlebot orientation pose 

  q_camera.normalize();

  Eigen::Matrix3f R_camera = q_camera.toRotationMatrix();   // convert a quaternion to a 3x3 rotation matrix:

  Eigen::Matrix4f camera_pose;
  camera_pose.setIdentity();   // Set to Identity 
  camera_pose.block<3,3>(0,0) = R_camera; 
  camera_pose.block<3,1>(0,3) = v_camera;

  //A(Aruco) *  camera_pose * robot_pose.inv 
  trans_camera_pose = camera_pose*robot_pose.inverse();

  //cout<<trans_camera_pose(0,3)<<" "<<trans_camera_pose(1,3)<<" " << trans_camera_pose(2,3)<<endl;
  //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  
}
void initRos(int argc, char** argv, string rgb_topic){
  ros::init(argc, argv, "marker_finder_ros");    //starting ros
  ros::start();

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, listenKeyboardSave);    //subscribing to string msg 

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, imageCallback);    //subscribing to rgb image
  
  ros::NodeHandle nn;
  ros::Subscriber odom_sub = nn.subscribe("odom", 1, odomCallback);

  ros::spin();  //"while true"
}