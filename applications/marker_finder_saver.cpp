//C++
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
float aruco_distance=0,aruco_marker_size = 0;
string camera_calibration_file="", rgb_topic="", aruco_dic="", aruco_poses_file="";


void imageCallback(const sensor_msgs::ImageConstPtr& msg); //listen to rgb image 
void rosMarkerFinder(cv::Mat rgb); //marker finder
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); //listen turtlebot odometry and make some robot transformations
void listenKeyboardCallback(const std_msgs::String::ConstPtr& msg); //listing to marker_goal topic
void initRos(int argc, char** argv,string rgb_topic); //Initializing ROS functions, as subs and ros spin
void loadParams(); //Load ConfigFile Params

int main(int argc, char** argv){  

  loadParams();
  marker_finder.markerParam(camera_calibration_file, aruco_marker_size, aruco_dic);

  for(int k=0; k<=254; k++){ //initializing markers
    all_markers[k].id = 0;
  }

  initRos(argc,argv,rgb_topic); //initializing ROS

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
  marker_finder.detectMarkers(rgb, trans_camera_pose,aruco_distance);   //Detect and get pose of all aruco markers

  for (size_t j = 0; j < marker_finder.markers_.size(); j++){
    all_markers[marker_finder.markers_[j].id].id = marker_finder.markers_[j].id;     //save all markers in a vetor 
    all_markers[marker_finder.markers_[j].id].x_pose = marker_finder.marker_point_poses_[j](0,0); //save marker position 
    all_markers[marker_finder.markers_[j].id].y_pose = marker_finder.marker_point_poses_[j](1,0);
    all_markers[marker_finder.markers_[j].id].z_pose = marker_finder.marker_point_poses_[j](2,0);
    marker_finder.markers_[j].draw(rgb, Scalar(0,0,255), 1);   //drawing markers in rgb image
    CvDrawingUtils::draw3dAxis(rgb, marker_finder.markers_[j], marker_finder.camera_params_); //drawing axis on window
    stringstream ss;
    ss << "m" << marker_finder.markers_[j].id;
  }
   
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
    arq.open(aruco_poses_file);
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
 * Load ConfigFile.yaml Params
 */
void loadParams(){
  string filename = "../../../src/ros_autonomous_robot/ConfigFile.yaml";
  FileStorage fs(filename,FileStorage::READ);  //Reading config file
  if(fs.isOpened() == false){
    cout<<"ConfigFile couldn't be opened, check if your path is right\n";
    exit(0);
  }
  try{//Loading Params
    fs["camera_calibration_file"] >> camera_calibration_file;
    fs["rgb_topic"] >> rgb_topic;
    fs["aruco_dic"] >> aruco_dic;
    fs["aruco_poses_file"] >> aruco_poses_file;
    fs["aruco_distance"] >> aruco_distance;
    fs["aruco_marker_size"] >> aruco_marker_size;
    //looking if any of the params was not loaded
    if(camera_calibration_file.empty() || rgb_topic.empty() || aruco_dic.empty() || aruco_poses_file.empty() || aruco_distance == 0  || aruco_marker_size == 0) throw 1;
   
    cout<<"Parameters loaded correctly\n\n"<<"camera_calibration_file: "<<camera_calibration_file<<endl;
    cout<<"rgb_topic: "<<rgb_topic<<endl<<"aruco_dic: "<<aruco_dic<<endl<<"aruco_pose_file: "<<aruco_poses_file<<endl;
    cout<<"aruco_distance: "<<aruco_distance<<endl<<"aruco_marker_size: "<<aruco_marker_size<<endl;
  }
  catch(int e){//If a param was not loaded, use the default
    cout<<"Coudn't load the params, at least one of the param names is wrong\n\n";
    cout<<"Using default values\ncamera_calibration_file: \"kinect_default.yaml\"\nrgb_topic: \"camera/rgb/image_raw\"\n";
    cout<<"aruco_dic: \"ARUCO\"\naruco_pose_file: \"aruco_poses\"\naruco_distance: 4\naruco_marker_size: 0.1778\n";
    camera_calibration_file = "kinect_default.yaml";
    rgb_topic = "camera/rgb/image_raw";
    aruco_dic = "ARUCO";
    aruco_poses_file = "aruco_poses";
    aruco_distance = 4;
    aruco_marker_size = 0.1778;
  }
}
