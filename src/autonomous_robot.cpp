#include <cstdio>
#include <cstdlib>
#include <fstream>
/// RGBD_RTK
#include <config_loader.h>
#include <marker_finder.h>
#include <reconstruction_visualizer.h>
/// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
/// Opencv
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
// Aruco
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
// Autonomous Robot
#include <goal.h>
#include <handleFiles.h>
#include <structures.h>

using namespace std;
using namespace cv;
using namespace aruco;

// aruco
MarkerDetector marker_detector;
CameraParameters camera_params;
float aruco_marker_size, aruco_max_distance, aruco_offset_distance;
string aruco_poses_file;
cv::Mat rgb;
tf::TransformBroadcaster *br;
Eigen::Affine3f I = Eigen::Affine3f::Identity();
Goal goal;

string listen_id;
int marker_id_asked;
int marker_id = 0;
vector<Pose> all_markers;
MarkerFinder marker_finder;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
// void markerGetCloser(int marker_id);
void listenKeyboardGoal(const std_msgs::String::ConstPtr &msg);
bool moveToGoal(Eigen::Quaternionf orientation, Eigen::Affine3f pose);
void loadMarkers(string aruco_poses_file);
void initRos(int argc, char **argv, string rgb_topic);
void loadParams();
void publishArucoTF();

int main(int argc, char **argv) {
    string camera_calibration_file, aruco_dic, rgb_topic;

    // Loading config variables
    ConfigLoader param_loader("../../../src/ros_autonomous_robot/config_files/ConfigFile.yaml");
    param_loader.checkAndGetString("camera_calibration_file", camera_calibration_file);
    param_loader.checkAndGetString("aruco_dic", aruco_dic);
    param_loader.checkAndGetString("rgb_topic", rgb_topic);
    param_loader.checkAndGetString("aruco_poses_file", aruco_poses_file);
    param_loader.checkAndGetFloat("aruco_offset_distance", aruco_offset_distance);
    param_loader.checkAndGetFloat("aruco_marker_size", aruco_marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", aruco_max_distance);

    marker_finder.markerParam(camera_calibration_file, aruco_marker_size, aruco_dic);

    initRos(argc, argv, rgb_topic);

    return 0;
}
/**
 * Ros Listener to rgb topic
 * @params reads a rgb message type ImageConstPtr
 */
void imageCallback(const sensor_msgs::ImageConstPtr &msgRGB) {

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    rgb = cv_ptrRGB->image;
    publishArucoTF();

    // Detect and get pose of all aruco markers
    marker_finder.detectMarkersPoses(rgb, I, aruco_max_distance);

    for (size_t j = 0; j < marker_finder.markers_.size(); j++) {
        if (marker_finder.markers_[j].id != marker_id) continue;

        marker_finder.markers_[j].draw(rgb, Scalar(0, 0, 255), 1); // drawing markers in rgb image
        CvDrawingUtils::draw3dAxis(rgb, marker_finder.markers_[j], marker_finder.camera_params_);
        stringstream ss;
        ss << "m" << marker_finder.markers_[j].id;
    }

    cv::imshow("OPENCV_WINDOW", rgb); // showing rgb image
    cv::waitKey(1);
}

/**
 * Listen to a marker number and send a goal to the robot goes to marker position
 * @Params Keeps listen to any string message sent in ROS
 */
void sendGoal(const std_msgs::String::ConstPtr &msg) {
    handleFiles.loadPoses(aruco_poses_file, aruco_off); // loading markers
    listen_id = msg->data.c_str();
    string::size_type sz;

    try {
        marker_id_asked = stoi(listen_id, &sz); // converting string to int
    } catch (std::invalid_argument &e) {
        cout << listen_id << " is not a number\n" << endl;
    }
    for (Pose pose : all_markers) {
        // validing a marker(a marker is valid if it was detected in any frame)
        if (pose.id == marker_id_asked) {
            ROS_INFO("[%s] is a valid marker", msg->data.c_str());
            marker_id = marker_id_asked;
            goal.send2dGoal(pose.affine_pose(0, 3), pose.affine_pose(1, 3),
                            Eigen::Quaterniond(1, 0, 0, 0));

        } else
            ROS_INFO("[%s] is not a valid marker", msg->data.c_str());
    }
}

/**
 * This function publishes the aruco markers tf related to 0,0
 */
void publishArucoTF() {
    br = new tf::TransformBroadcaster();
    tf::Transform transform;
    for (Pose pose : all_markers) {
        if (pose.id == 0) continue; // if marker not found continue
        // set the xyz and rotation pose
        transform.setOrigin(
            tf::Vector3(pose.affine_pose(0, 3), pose.affine_pose(1, 3), pose.affine_pose(2, 3)));
        transform.setRotation(
            tf::Quaternion(pose.w_rotation, pose.x_rotation, pose.y_rotation, pose.z_rotation));
        string aruco_tf = "aruco" + to_string(j); // set aruco name
        // broadcasting to tf related to odom
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", aruco_tf));
    }
}

/**
 * Initialize ROS
 */
void initRos(int argc, char **argv, string rgb_topic) {
    ros::init(argc, argv, "autonomous_robot");
    ros::start();

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("marker_goal", 1000, sendGoal);

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, imageCallback);

    ros::spin(); //"while true"
}
