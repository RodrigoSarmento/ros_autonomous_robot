#include <cstdio>
#include <cstdlib>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "config_loader.h"


using namespace std;
using namespace cv;
/**
 * Load configfile params
 * @Params string , path where the configfile is
 */
void ConfigLoader::loadParams(string filename){
  FileStorage fs(filename,FileStorage::READ);  //Reading config file
  if(fs.isOpened() == false){
    cout<<"ConfigFile couldn't be opened, check if your path is right\n";
    exit(0);
  }

  try{//Loading Params
    fs["camera_calibration_file"] >> camera_calibration_file_;
    fs["rgb_topic"] >> rgb_topic_;
    fs["depth_topic"] >> depth_topic_;
    fs["aruco_dic"] >> aruco_dic_;
    fs["aruco_poses_file"] >> aruco_poses_file_;
    fs["aruco_max_distance"] >> aruco_max_distance_;
    fs["aruco_marker_size"] >> aruco_marker_size_;
    //looking if any of the params was not loaded
    if(camera_calibration_file_.empty()) throw 1;
    if(rgb_topic_.empty()) throw 2;
    if(depth_topic_.empty()) throw 3;
    if(aruco_dic_.empty()) throw 4; 
    if(aruco_poses_file_.empty()) throw 5;
    if(aruco_max_distance_ == 0) throw 6;
    if(aruco_marker_size_ == 0) throw 7;
   
    cout<<"Params load successfully\n"<<"camera_calibration_file: "<<camera_calibration_file_<<endl;
    cout<<"rgb_topic: "<<rgb_topic_<<endl<<"depth_topic: "<<depth_topic_<<endl<<"aruco_dic: "<<aruco_dic_<<endl<<"aruco_pose_file: "<<aruco_poses_file_<<endl;
    cout<<"aruco_max_distance: "<<aruco_max_distance_<<endl<<"aruco_marker_size: "<<aruco_marker_size_<<endl;
  }
  catch(int e){//If a param was not loaded, use the default
    cout<<"Couldn't load params\n";
    switch (e)
    {
    case 1:
      cout<<"Calibration file is empty\nTrying to use the default path:../../../src/ros_autonomous_robot/kinect_default.yaml\n";
      camera_calibration_file_ = "../../../src/ros_autonomous_robot/kinect_default.yaml";
      break;
    case 2:
      cout<<"rgb_topic is empty\nTrying to use the default topic: camera/rgb/image_raw\n";
      rgb_topic_ = "camera/rgb/image_raw";
      break;
    case 3:
      cout<<"depth_topic is empty\nTrying to use the default topic: camera/depth/image_raw\n";
      depth_topic_ = "camera/depth/image_raw";
      break;
    case 4:
      cout<<"aruco_dic is empty\nTrying to use the dictionarty: ARUCO\n";
      aruco_dic_ = "ARUCO";
      break;
    case 5:
      cout<<"aruco_poses_file is empty\nTrying to use the default file name: aruco_poses\n";
      aruco_poses_file_ = "aruco_poses";
      break;
    case 6:
      cout<<"aruco_max_distance is empty\nTrying to use the default value: 4\n";
      aruco_max_distance_ = 4;
      break;
    case 7:
      cout<<"aruco_marker_size_ is empty\nTrying to use the default value: 0.1778\n";
      aruco_marker_size_ = 0.1778;
      break;
    }
  }
}