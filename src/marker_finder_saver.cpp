// C++
#include <cstdio>
#include <cstdlib>
#include <fstream>
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
#include </usr/local/include/config_loader.h>
#include <geometry.h>
#include <klt_tracker.h>
#include <marker_finder.h>
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>

using namespace std;
using namespace cv;
using namespace aruco;

// Struct of markers, a combination of marker id and xyz position
struct markerFound {
    int id;
    float x_pose;
    float y_pose;
    float z_pose;
    float x_rotation;
    float y_rotation;
    float z_rotation;
    float w_rotation;
};

MarkerFinder marker_finder; // markerfinder
float aruco_marker_size, aruco_max_distance;
Eigen::Affine3f trans_camera_pose; // turtlebot pose
markerFound all_markers[255];      // list of marker struct
string aruco_poses_file;
int id = -1;
tf::TransformBroadcaster *br; // Broadcaster for tf aruco
Eigen::Quaternionf q_aruco;   // Quaternion of aruco pose

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void rosMarkerFinder(cv::Mat rgb);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void listenKeyboardCallback(const std_msgs::String::ConstPtr &msg);
void initRos(int argc, char **argv, string rgb_topic);
void loadParams();
void publishArucoTF();

int main(int argc, char **argv) {
    string camera_calibration_file, aruco_dic, rgb_topic;

    ConfigLoader param_loader(
        "../../../src/ros_autonomous_robot/config_files/ConfigFile.yaml"); // Load config file param
    param_loader.checkAndGetString("camera_calibration_file", camera_calibration_file);
    param_loader.checkAndGetString("aruco_dic", aruco_dic);
    param_loader.checkAndGetString("rgb_topic", rgb_topic);
    param_loader.checkAndGetString("aruco_poses_file", aruco_poses_file);
    param_loader.checkAndGetFloat("aruco_marker_size", aruco_marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", aruco_max_distance);

    marker_finder.markerParam(camera_calibration_file, aruco_marker_size, aruco_dic);

    for (int k = 0; k < 255; k++) { // initializing markers
        all_markers[k].id = 0;
    }

    initRos(argc, argv, rgb_topic); // initializing ROS

    return 0;
}
/**
 * ROS Callback listen to rgb topic
 * @param sensor_msgs::ImageConstPtr image message
 */
void imageCallback(const sensor_msgs::ImageConstPtr &msgRGB) {

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB); // trying convert rgb
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    rosMarkerFinder(cv_ptrRGB->image); // calling marker finder function
}

/**
 * Looks foward ARUCO markers and saves in a  list of markerFound structure
 * @param cv::Mat image
 */
void rosMarkerFinder(cv::Mat rgb) {

    marker_finder.detectMarkersPoses(
        rgb, trans_camera_pose, aruco_max_distance); // Detect and get pose of all aruco markers

    for (size_t j = 0; j < marker_finder.markers_.size(); j++) {
        id = marker_finder.markers_[j].id;
        all_markers[id].id = id; // save all markers in a vetor
        all_markers[id].x_pose = marker_finder.marker_poses_[j](0, 3); // save marker position
        all_markers[id].y_pose = marker_finder.marker_poses_[j](1, 3);
        all_markers[id].z_pose = marker_finder.marker_poses_[j](2, 3);
        q_aruco = marker_finder.marker_poses_[j].rotation();
        all_markers[id].x_rotation = q_aruco.x();
        all_markers[id].y_rotation = q_aruco.y();
        all_markers[id].z_rotation = q_aruco.z();
        all_markers[id].w_rotation = q_aruco.w();

        marker_finder.markers_[j].draw(rgb, Scalar(0, 0, 255), 1); // drawing markers in rgb image
        CvDrawingUtils::draw3dAxis(rgb, marker_finder.markers_[j],
                                   marker_finder.camera_params_); // drawing axis on window
        stringstream ss;
        ss << "m" << id;
    }

    publishArucoTF(); // Publishing aruco psoe

    cv::imshow("OPENCV_WINDOW", rgb); // showing rgb image
    cv::waitKey(1);
}

/**
 * Listen to robot odom and calculates camera position related to 0,0 of map
 * @param nav_msgs::Odometry::ConstPtr message
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

    Eigen::Vector3f v_robot(msg->pose.pose.position.x, msg->pose.pose.position.y,
                            0); // subscribing turtlebot pose
    Eigen::Quaternionf q_robot(
        (msg->pose.pose.orientation.w), msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        (msg->pose.pose.orientation.z)); /// subscribing turtlebot orientation pose

    Eigen::Matrix3f R_robot =
        q_robot.normalized().toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix:

    Eigen::Matrix4f robot_pose;
    robot_pose.setIdentity(); // Set to Identity
    robot_pose.block<3, 3>(0, 0) = R_robot;
    robot_pose.block<3, 1>(0, 3) = v_robot;

    Eigen::Vector3f v_camera(-0.087, -0.0125, 0.2972); // subscribing turtlebot pose
    Eigen::Quaternionf q_camera(0.5, -0.5, 0.5, -0.5); /// subscribing turtlebot orientation pose

    Eigen::Matrix3f R_camera =
        q_camera.normalized().toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix:

    Eigen::Matrix4f camera_pose;
    camera_pose.setIdentity(); // Set to Identity
    camera_pose.block<3, 3>(0, 0) = R_camera;
    camera_pose.block<3, 1>(0, 3) = v_camera;

    trans_camera_pose = camera_pose.inverse() *
                        robot_pose.inverse(); // calculating Tcamera pose in relation to 0,0 global

    // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]",
    // msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x,
    // msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",
    // msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

/**
 * ROS Callback that listen to a string in order to save aruco markers in file
 * @param String::ConstPrt message
 */
void listenKeyboardCallback(const std_msgs::String::ConstPtr &msg) {
    string listen = msg->data.c_str(); // reading msg that was sent by marker_goal topic
    int cont = 0;
    if (listen.compare("s") == 0) { // validing if string msg is 's'
        ofstream arq;
        arq.open(aruco_poses_file);
        for (int k = 0; k <= 254; k++) {
            if (all_markers[k].id == 0) continue;
            cont++;
        } // writing in the first line the number of markers that were found

        arq << cont << endl;
        for (int k = 0; k <= 254; k++) {
            if (all_markers[k].id == 0) continue;
            arq << all_markers[k].id << " " << all_markers[k].x_pose << " " << all_markers[k].y_pose
                << " " << all_markers[k].z_pose << " " << all_markers[k].x_rotation << " "
                << all_markers[k].y_rotation << " " << all_markers[k].z_rotation << " "
                << all_markers[k].w_rotation << endl; // saving all markers in "all_markers.txt"
        }
        ROS_INFO("Markers Saved");
    } else
        ROS_INFO("[%s] is not a valid input, use 's' to save all markers", msg->data.c_str());
}

/**
 * This function publishes the aruco markers tf related to 0,0
 */
void publishArucoTF() {
    br = new tf::TransformBroadcaster();
    tf::Transform transform;
    for (int j = 1; j < 255; j++) {
        if (all_markers[j].id == 0) continue; // if marker not found continue
        // set the xyz and rotation pose
        transform.setOrigin(
            tf::Vector3(all_markers[j].x_pose, all_markers[j].y_pose, all_markers[j].z_pose));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        string aruco_tf = "aruco" + to_string(j); // set aruco name
        // broadcasting to tf related to odom
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", aruco_tf));
    }
}

/**
 * Initialize ROS
 */
void initRos(int argc, char **argv, string rgb_topic) {
    ros::init(argc, argv, "marker_finder_ros"); // starting ros
    ros::start();

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("marker_goal", 1000,
                                      listenKeyboardCallback); // subscribing to marker_goal topic

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub =
        it.subscribe(rgb_topic, 1, imageCallback); // subscribing to rgb image

    ros::NodeHandle nn;
    ros::Subscriber odom_sub = nn.subscribe("odom", 1000, odomCallback); // subscribing to odom
                                                                         // topic

    ros::spin(); // leting ROS doing what he needs to do
}
