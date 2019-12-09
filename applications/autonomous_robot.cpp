#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <config_loader.h>
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
#include <sensor_msgs/Image.h>
///Opencv
#include <opencv2/highgui/highgui.hpp>
//Aruco
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>


using namespace std;
using namespace cv;
using namespace aruco;


//aruco
MarkerDetector marker_detector;
CameraParameters camera_params;
vector<Marker> markers;

ConfigLoader Loader;

struct markerFound{
  int id;
  float x_pose;
  float y_pose;
  float z_pose;
};

string listen_id;
int listen_id_to_int;
markerFound all_markers[255];

void imageCallback(const sensor_msgs::ImageConstPtr& msg); //subscribe to rgb image
void markerFinder(cv::Mat rgb); //marker finder
void listenKeyboardGoal(const std_msgs::String::ConstPtr& msg); //listening keyboard input for navigation
bool moveToGoal(double xGoal, double yGoal); //moving autonomous to a place
void loadMarkers(string aruco_poses_file); //open all_markers.txt
void initRos(int argc, char** argv, string rgb_topic);
void loadParams(); //Load ConfigFile Params


int main(int argc, char** argv){    
  Loader.loadParams("../../../src/ros_autonomous_robot/ConfigFile.yaml"); //Load config file param

  camera_params.readFromXMLFile(Loader.camera_calibration_file_);    //aruco params 
  marker_detector.setDictionary(Loader.aruco_dic_, 0);

  for(int k=0; k<=254; k++){ //initializing markers
    all_markers[k].id = 0;
  }
  initRos(argc,argv,Loader.rgb_topic_);

  return 0;
 }
/**
 * Ros Listener to rgb topic
 * @Params reads a rgb message
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msgRGB){
  
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try{
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  markerFinder(cv_ptrRGB->image); //calling marker finder funcition
}

/**
 * Detects a aruco marker and draw it on image
 * @Params rgb image as cv::mat
 */ 
void markerFinder(cv::Mat rgb ){

  marker_detector.detect(rgb, markers, camera_params, Loader.aruco_marker_size_);   //Detect and view Aruco markers

  for (size_t j = 0; j < markers.size(); j++){
    markers[j].draw(rgb, Scalar(0,0,255), 1);   //drawing markers in rgb image
    CvDrawingUtils::draw3dAxis(rgb, markers[j], camera_params);
    stringstream ss;
    ss << "m" << markers[j].id;
  }
   
  cv::imshow("OPENCV_WINDOW", rgb);  //showing rgb image
  cv::waitKey(1);

}
/**
 * Listen to a marker number and send a goal to the robot goes to marker position
 * @Params Keeps listen to any string message sent in ROS
 */
void listenKeyboardCallback(const std_msgs::String::ConstPtr& msg){

  loadMarkers(Loader.aruco_poses_file_);//loading markers
  listen_id = msg->data.c_str();
  string::size_type sz; 

  try{
    listen_id_to_int = stoi(listen_id,&sz);//converting string to int
  } 
  catch(std::invalid_argument& e){
    cout<<listen_id<< " is not a number\n"<<endl;
  }
  if(all_markers[listen_id_to_int].id != 0){   //validing a marker(a marker is valid if it was detected in any frame)
    ROS_INFO("[%s] is a valid marker", msg->data.c_str());
    moveToGoal(all_markers[listen_id_to_int].x_pose, all_markers[listen_id_to_int].y_pose);
  }

  else 
    ROS_INFO("[%s] is not a valid marker", msg->data.c_str());
}
/**
 * Send a goal to a position in map
 * @Params x and y position
 * @Return boolean if the robot reached or not the position
 */
bool moveToGoal(double xGoal, double yGoal){

   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);    //define a client for to send goal requests to the move_base server through a SimpleActionClient


   while(!ac.waitForServer(ros::Duration(5.0))){    //wait for the action server to come up
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   goal.target_pose.header.frame_id = "map";     //set up the frame parameters
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  xGoal;
   goal.target_pose.pose.position.y =  yGoal;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the destination");
      return false;
   }
}
/**
 * Read all know aruco markers
 */

void loadMarkers(string aruco_poses_file){
  int markers_number = 0, id = 0;
  float x = 0, y = 0, z = 0;
  fstream arq;
  arq.open(aruco_poses_file);//open .txt
  arq >> markers_number;

  //saving markers information in vector "all_markers"
  for(int i = 0; i < markers_number; i++){
    arq >> id >> x >> y >>z;
    all_markers[id].id = id;
    all_markers[id].x_pose = x;
    all_markers[id].y_pose = y;
    all_markers[id].z_pose = z;

  }
}
/**
 * Initialize ROS 
 */
void initRos(int argc, char** argv, string rgb_topic){
  ros::init(argc, argv, "autonomous_robot");
  ros::start();

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("marker_goal", 1000, listenKeyboardCallback); //listning navigation goal
  
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, imageCallback);    //subscribing to rgb image

  ros::spin();  //"while true"
}
