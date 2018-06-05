#include <cstdio>
#include <cstdlib>
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
float marker_size;

int i=0;


struct markerFound{
  int id;
  double x_pose;
  double y_pose;
  double z_pose;
};

string listen_id;
int listen_id_to_int;
markerFound all_markers[255];

void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD); // listening rgbd sensor 
void rosMarkerFinder(cv::Mat rgb , cv::Mat depth); //marker finder
void listenKeyboardGoal(const std_msgs::String::ConstPtr& msg); //listening keyboard input for navigation
bool goalReached = false;
bool moveToGoal(double xGoal, double yGoal); //moving autonomous to a place


int main(int argc, char** argv){    
  
  string rgb_topic,depth_topic;
  rgb_topic = "camera/rgb/image_raw";
  depth_topic = "camera/depth/image_raw";
  if(argc != 5 && argc !=3){
    fprintf(stderr, "Usage: %s <camera calibration file> <marker size> optional: <rgb_topic> <depth_topic>....bye default : camera/rgb/image_raw and camera/depth/image_raw\n", argv[0]);
    exit(0);
  }
  if(argc == 3){
    printf(" By defult using camera/rgb/image_raw and camera/depth/image_raw as ros topics\n");  
  }
   if(argc == 5){
     rgb_topic = argv[3];
     depth_topic = argv[4];  
  }
 


  camera_params.readFromXMLFile(argv[1]);
  marker_size = stof(argv[2]);
  marker_detector.setDictionary("ARUCO_MIP_36h12", 0);

  //ROS steps
  for(int k=0; k>=255; k++){
    all_markers[k].id = 0;
  }

  ros::init(argc, argv, "marker_finder_ros");
  ros::start();

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, listenKeyboardGoal);
  
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  //ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub,depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
 }

void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD){

  // Copy the ros image message to cv::Mat.
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
  rosMarkerFinder(cv_ptrRGB->image, cv_ptrD->image);
}

void rosMarkerFinder(cv::Mat rgb , cv::Mat depth){

  //Detect and view Aruco markers
  marker_detector.detect(rgb, markers, camera_params, marker_size); 
  
  for (size_t j = 0; j < markers.size(); j++){
    //save all markers in a vetor 
    all_markers[markers[j].id].id = markers[j].id;
    markers[j].draw(rgb, Scalar(0,0,255), 1);
    //use to put names on ids cout<<markers[j].id<<" ";
    CvDrawingUtils::draw3dAxis(rgb, markers[j], camera_params);
    stringstream ss;
    ss << "m" << markers[j].id;
  }

  depth = depth/5;
  cv::imshow("OPENCV_WINDOW", rgb);
  cv::imshow("OPENCV_WINDOW_DEPTH", depth);
  cv::waitKey(1);

  i++;
}


void listenKeyboardGoal(const std_msgs::String::ConstPtr& msg){
  listen_id = msg->data.c_str();
  string::size_type sz; 
  //converting string to int
  listen_id_to_int = stoi(listen_id,&sz);

  //validing a marker(a merker is valid if it was detected in any frame)
  if(all_markers[listen_id_to_int].id != 0){
    ROS_INFO("[%s] is a valid marker", msg->data.c_str());
    moveToGoal(all_markers[listen_id_to_int].x_pose, all_markers[listen_id_to_int].y_pose);
  }

  else 
    ROS_INFO("[%s] is not a valid marker", msg->data.c_str());
}
bool moveToGoal(double xGoal, double yGoal){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
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