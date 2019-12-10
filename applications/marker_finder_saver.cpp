//C++
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <string>
#include <config_loader.h>

///ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
///Opencv
#include <opencv2/highgui/highgui.hpp>
//Aruco
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <marker_finder.h>

using namespace std;
using namespace cv;
using namespace aruco;

ConfigLoader Loader;

//Struct of markers, a combination of marker id and xyz position
struct markerFound{
  int id;
  float x_pose;
  float y_pose;
  float z_pose;
};

MarkerFinder marker_finder; //markerfinder
Eigen::Affine3f trans_camera_pose; //turtlebot pose
markerFound all_markers[255]; //list of marker struct
string aruco_tf = "";
int id = -1;


void imageCallback(const sensor_msgs::ImageConstPtr& msg); //listen to rgb image 
void rosMarkerFinder(cv::Mat rgb); //marker finder
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); //listen turtlebot odometry and make some robot transformations
void listenKeyboardCallback(const std_msgs::String::ConstPtr& msg); //listing to marker_goal topic
void initRos(int argc, char** argv,string rgb_topic); //Initializing ROS functions, as subs and ros spin
void loadParams(); //Load ConfigFile Params
void publishArucoTF(); 

int main(int argc, char** argv){  

  Loader.loadParams("../../../src/ros_autonomous_robot/ConfigFile.yaml"); //Load config file param
  marker_finder.markerParam(Loader.camera_calibration_file_, Loader.aruco_marker_size_, Loader.aruco_dic_);

  for(int k=0; k<=254; k++){ //initializing markers
    all_markers[k].id = 0;
  }

  initRos(argc,argv,Loader.rgb_topic_); //initializing ROS

  return 0;
 }
/**
 * ROS Callback listen to rgb topic
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msgRGB){
  
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try{
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB); //trying convert rgb 
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  rosMarkerFinder(cv_ptrRGB->image); //calling marker finder function
}

/**
 * Looks foward ARUCO markers and saves in a  list of markerFound structure
 */
void rosMarkerFinder(cv::Mat rgb){
  marker_finder.detectMarkers(rgb, trans_camera_pose,Loader.aruco_minimum_distance_);   //Detect and get pose of all aruco markers

  for (size_t j = 0; j < marker_finder.markers_.size(); j++){
    id = marker_finder.markers_[j].id;
    all_markers[id].id = id;     //save all markers in a vetor 
    all_markers[id].x_pose = marker_finder.marker_point_poses_[j](0,0); //save marker position 
    all_markers[id].y_pose = marker_finder.marker_point_poses_[j](1,0);
    all_markers[id].z_pose = marker_finder.marker_point_poses_[j](2,0);
    marker_finder.markers_[j].draw(rgb, Scalar(0,0,255), 1);   //drawing markers in rgb image
    CvDrawingUtils::draw3dAxis(rgb, marker_finder.markers_[j], marker_finder.camera_params_); //drawing axis on window
    stringstream ss;
    ss << "m" << id;

  }
  publishArucoTF();

  cv::imshow("OPENCV_WINDOW", rgb);  //showing rgb image
  cv::waitKey(1);

}

/**
 * Listen to robot odom and calculates camera position related to 0,0 of map
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

  Eigen::Vector3f v_robot(msg->pose.pose.position.x,msg->pose.pose.position.y,0); //subscribing turtlebot pose 
  Eigen::Quaternionf q_robot((msg->pose.pose.orientation.w), msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, (msg->pose.pose.orientation.z)); ///subscribing turtlebot orientation pose 
  

  Eigen::Matrix3f R_robot = q_robot.normalized().toRotationMatrix();   // convert a quaternion to a 3x3 rotation matrix:

  Eigen::Matrix4f robot_pose;
  robot_pose.setIdentity();   // Set to Identity 
  robot_pose.block<3,3>(0,0) = R_robot; 
  robot_pose.block<3,1>(0,3) = v_robot;

  Eigen::Vector3f v_camera(-0.087, -0.0125, 0.2972); //subscribing turtlebot pose 
  Eigen::Quaternionf q_camera(0.5, -0.5, 0.5, -0.5); ///subscribing turtlebot orientation pose 


  Eigen::Matrix3f R_camera =  q_camera.normalized().toRotationMatrix();   // convert a quaternion to a 3x3 rotation matrix:

  Eigen::Matrix4f camera_pose;
  camera_pose.setIdentity();   // Set to Identity 
  camera_pose.block<3,3>(0,0) = R_camera; 
  camera_pose.block<3,1>(0,3) = v_camera;

  trans_camera_pose = camera_pose.inverse()*robot_pose.inverse(); // calculating Tcamera pose in relation to 0,0 global

  //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  
  //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

/**
 * ROS Callback that listen to a string in order to save aruco markers in file
 */
void listenKeyboardCallback(const std_msgs::String::ConstPtr& msg){
  string listen = msg->data.c_str(); //reading msg that was sent by marker_goal topic
  int cont=0;
  if(listen.compare("s") == 0){  //validing if string msg is 's'
    ofstream arq;
    arq.open(Loader.aruco_poses_file_);
    for(int k=0; k<=254; k++){
      if(all_markers[k].id==0) continue;
        cont ++;
    } //writing in the first line the number of markers that were found

    arq<<cont<<endl;
    for(int k=0; k<=254; k++){
      if(all_markers[k].id==0) continue;
        arq<<all_markers[k].id<<" "<<all_markers[k].x_pose<<" "<<all_markers[k].y_pose <<" "<<all_markers[k].z_pose << endl;   //saving all markers in "all_markers.txt"
    }
    ROS_INFO("Markers Saved");
  }
  else 
    ROS_INFO("[%s] is not a valid input, use 's' to save all markers", msg->data.c_str());
}

/**
 * Initialize ROS
 */
void initRos(int argc, char** argv, string rgb_topic){
  ros::init(argc, argv, "marker_finder_ros");    //starting ros
  ros::start();

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("marker_goal", 1000, listenKeyboardCallback);    //subscribing to marker_goal topic 

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, imageCallback);    //subscribing to rgb image
  
  ros::NodeHandle nn;
  ros::Subscriber odom_sub = nn.subscribe("odom", 1000, odomCallback); //subscribing to odom topic

  ros::spin();  // leting ROS doing what he needs to do
}

/**
 * THis function publishes the aruco markers tf related to 0,0
 */
void publishArucoTF(){
  tf::TransformBroadcaster br;
  tf::Transform transform;
  for (int j = 1; j < 255; j++){
    if(all_markers[j].id == 0) continue;
    transform.setOrigin(tf::Vector3(all_markers[j].x_pose, all_markers[j].y_pose, all_markers[j].z_pose));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    aruco_tf = "aruco" + to_string(j);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", aruco_tf));
  }
}