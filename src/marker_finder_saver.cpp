// C++
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
/// ROS
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
/// Opencv
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
// RGBD_RTK
#include <config_loader.h>
#include <marker_finder.h>
#include <reconstruction_visualizer.h>
// Autonomous Robot
#include <handleFiles.h>
#include <structures.h>

using namespace std;
using namespace cv;
using namespace aruco;

MarkerFinder marker_finder;
Eigen::Affine3f trans_camera_pose; // Turtlebot pose
Eigen::Quaternionf q_aruco;        // Quaternion of aruco pose
tf::TransformBroadcaster *br;      // Broadcaster for tf aruco
vector<Pose> all_markers;          // List of marker struct
string aruco_poses_file;
int id = -1;
float aruco_marker_size, aruco_max_distance;
HandleFiles handleFiles;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void rosMarkerFinder(cv::Mat rgb);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void savePosesInFile(const std_msgs::String::ConstPtr &msg);
void initRos(int argc, char **argv, string rgb_topic);
void loadParams();
void publishArucoTF();

int main(int argc, char **argv) {
    string camera_calibration_file, aruco_dic, rgb_topic;
    // Load config file param
    ConfigLoader param_loader("../../../src/ros_autonomous_robot/config_files/ConfigFile.yaml");
    param_loader.checkAndGetString("camera_calibration_file", camera_calibration_file);
    param_loader.checkAndGetString("aruco_dic", aruco_dic);
    param_loader.checkAndGetString("rgb_topic", rgb_topic);
    param_loader.checkAndGetString("aruco_poses_file", aruco_poses_file);
    param_loader.checkAndGetFloat("aruco_marker_size", aruco_marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", aruco_max_distance);

    marker_finder.markerParam(camera_calibration_file, aruco_marker_size, aruco_dic);

    initRos(argc, argv, rgb_topic); // Initializing ROS

    return 0;
}
/**
 * ROS Callback listen to rgb topic
 * @param sensor_msgs::ImageConstPtr image message
 */
void imageCallback(const sensor_msgs::ImageConstPtr &msgRGB) {
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB); // Trying convert rgb
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    rosMarkerFinder(cv_ptrRGB->image); // Calling marker finder function
}

/**
 * Looks forward ARUCO markers and saves in a  list of markerFound structure
 * @param cv::Mat image
 */
void rosMarkerFinder(cv::Mat rgb) {
    printf("89\n");
    // Detect and get pose of all aruco markers
    marker_finder.detectMarkersPoses(rgb, trans_camera_pose, aruco_max_distance);
    Pose aruco_pose;
    printf("93\n");

    for (size_t j = 0; j < marker_finder.markers_.size(); j++) {
        printf("96\n");
        id = marker_finder.markers_[j].id;
        aruco_pose.id = id;                                  // Save all markers in a vector
        aruco_pose.x = marker_finder.marker_poses_[j](0, 3); // Save marker position
        aruco_pose.y = marker_finder.marker_poses_[j](1, 3);
        aruco_pose.z = marker_finder.marker_poses_[j](2, 3);
        printf("101\n");
        q_aruco = marker_finder.marker_poses_[j].rotation();
        aruco_pose.x_rotation = q_aruco.x();
        aruco_pose.y_rotation = q_aruco.y();
        aruco_pose.z_rotation = q_aruco.z();
        aruco_pose.w_rotation = q_aruco.w();
        printf("106\n");
        all_markers.push_back(aruco_pose);
        printf("109\n");

        marker_finder.markers_[j].draw(rgb, Scalar(0, 0, 255), 1); // Drawing markers in rgb image
        // Drawing axis on window
        CvDrawingUtils::draw3dAxis(rgb, marker_finder.markers_[j], marker_finder.camera_params_);
        stringstream ss;
        ss << "m" << id;
    }

    publishArucoTF(); // Publishing aruco pose

    cv::imshow("OPENCV_WINDOW", rgb); // Showing rgb image
    cv::waitKey(1);
}

/**
 * Listen to robot odom and calculates camera position related to 0,0 of map
 * @param nav_msgs::Odometry::ConstPtr message
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // Subscribing turtlebot pose
    Eigen::Vector3f v_robot(msg->pose.pose.position.x, msg->pose.pose.position.y, 0);
    // Subscribing turtlebot orientation pose
    Eigen::Quaternionf q_robot((msg->pose.pose.orientation.w), msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y, (msg->pose.pose.orientation.z));
    // Convert a quaternion to a 3x3 rotation matrix:
    Eigen::Matrix3f R_robot = q_robot.normalized().toRotationMatrix();

    Eigen::Matrix4f robot_pose;
    robot_pose.setIdentity(); // Set to Identity
    robot_pose.block<3, 3>(0, 0) = R_robot;
    robot_pose.block<3, 1>(0, 3) = v_robot;

    Eigen::Vector3f v_camera(-0.087, -0.0125, 0.2972); // Subscribing turtlebot pose
    Eigen::Quaternionf q_camera(0.5, -0.5, 0.5, -0.5); // Subscribing turtlebot orientation pose

    // Convert a quaternion to a 3x3 rotation matrix:
    Eigen::Matrix3f R_camera = q_camera.normalized().toRotationMatrix();

    Eigen::Matrix4f camera_pose;
    camera_pose.setIdentity(); // Set to Identity
    camera_pose.block<3, 3>(0, 0) = R_camera;
    camera_pose.block<3, 1>(0, 3) = v_camera;

    trans_camera_pose = camera_pose.inverse() * robot_pose.inverse();
    // Calculating Transformation camera pose in relation to 0,0 global

    /* ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]",
       msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
       ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x,
       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
       ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",
       msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    */
}

/**
 * ROS Callback that listen to a string in order to save aruco markers in file
 * @param String::ConstPrt message
 */
void savePosesInFile(const std_msgs::String::ConstPtr &msg) {
    string listen = msg->data.c_str(); // Reading msg that was sent by marker_goal topic
                                       // Validing if string msg is 's'
    if (listen.compare("s") == 0)
        handleFiles.savePoses(all_markers, aruco_poses_file);
    else
        ROS_INFO("[%s] is not a valid input, use 's' to save all markers", msg->data.c_str());
}

/**
 * This function publishes the aruco markers tf related to 0,0
 */
void publishArucoTF() {
    br = new tf::TransformBroadcaster();
    tf::Transform transform;
    for (int j = 1; j < 255; j++) {
        if (all_markers[j].id == 0) continue; // If marker not found continue
        // Set the xyz and rotation pose
        transform.setOrigin(tf::Vector3(all_markers[j].x, all_markers[j].y, all_markers[j].z));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        string aruco_tf = "aruco" + to_string(j); // Set aruco name
        // Broadcasting to tf related to odom
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", aruco_tf));
    }
}

/**
 * Initialize ROS
 */
void initRos(int argc, char **argv, string rgb_topic) {
    ros::init(argc, argv, "marker_finder_ros"); // Starting ros
    ros::start();

    // Subscribing to marker_goal topic
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("marker_goal", 1000, savePosesInFile);
    // Subscribing to rgb image
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, imageCallback);

    ros::NodeHandle nn;
    ros::Subscriber odom_sub = nn.subscribe("odom", 1000, odomCallback); // Subscribing to odom

    ros::spin(); // Leting ROS doing what he needs to do
}
