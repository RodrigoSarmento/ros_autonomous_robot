#include <cstdio>
#include <cstdlib>
#include <fstream>
/// RGBD_RTK
#include </usr/local/include/config_loader.h>
#include <geometry.h>
#include <marker_finder.h>
#include <reconstruction_visualizer.h>
#include <rgbd_loader.h>
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

#include <goal.h>

using namespace std;
using namespace cv;
using namespace aruco;

// aruco
MarkerDetector marker_detector;
CameraParameters camera_params;
vector<Marker> markers;
float aruco_marker_size, aruco_max_distance, aruco_close_distance;
string aruco_poses_file;
cv::Mat rgb;
tf::TransformBroadcaster *br;
Eigen::Affine3f I = Eigen::Affine3f::Identity();
Goal goal;

struct markerFound {
    int id;
    float x_pose;
    float y_pose;
    float z_pose;
    float x_rotation;
    float y_rotation;
    float z_rotation;
    float w_rotation;
    Eigen::Vector3f vector_pose;
    Eigen::Quaternionf orientation;
    Eigen::Affine3f pose;
};

string listen_id;
int marker_id_asked;
int marker_id = 0;
markerFound all_markers[255];
MarkerFinder marker_finder;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void markerGetCloser(int marker_id);
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
    param_loader.checkAndGetFloat("aruco_close_distance", aruco_close_distance);
    param_loader.checkAndGetFloat("aruco_marker_size", aruco_marker_size);
    param_loader.checkAndGetFloat("aruco_max_distance", aruco_max_distance);

    marker_finder.markerParam(camera_calibration_file, aruco_marker_size, aruco_dic);

    for (int k = 0; k <= 254; k++) { // initializing markers
        all_markers[k].id = 0;
    }
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
void listenKeyboardCallback(const std_msgs::String::ConstPtr &msg) {
    loadMarkers(aruco_poses_file); // loading markers
    listen_id = msg->data.c_str();
    string::size_type sz;

    try {
        marker_id_asked = stoi(listen_id, &sz); // converting string to int
    } catch (std::invalid_argument &e) {
        cout << listen_id << " is not a number\n" << endl;
    }
    // validing a marker(a marker is valid if it was detected in any frame)
    if (all_markers[marker_id_asked].id != 0) {
        ROS_INFO("[%s] is a valid marker", msg->data.c_str());
        marker_id = marker_id_asked;
        goal.send2dGoal(all_markers[marker_id_asked].pose(0, 3),
                        all_markers[marker_id_asked].pose(1, 3), Eigen::Quaterniond(1, 0, 0, 0));

    } else
        ROS_INFO("[%s] is not a valid marker", msg->data.c_str());
}

/**
 * Read all know aruco markers
 */

void loadMarkers(string aruco_poses_file) {
    int markers_number = 0, id = 0;
    float x = 0, y = 0, z = 0, x_r = 0, y_r = 0, z_r = 0, w_r = 0;
    fstream arq;
    arq.open(aruco_poses_file); // open .txt
    arq >> markers_number;

    // saving markers information in vector "all_markers"
    for (int i = 0; i < markers_number; i++) {
        arq >> id >> x >> y >> z >> x_r >> y_r >> z_r >> w_r;
        all_markers[id].id = id;
        all_markers[id].x_pose = x;
        all_markers[id].y_pose = y;
        all_markers[id].z_pose = z;
        all_markers[id].x_rotation = x_r;
        all_markers[id].y_rotation = y_r;
        all_markers[id].z_rotation = z_r;
        all_markers[id].w_rotation = w_r;

        Eigen::Vector3f v(x, y, z);
        Eigen::Quaternionf q(w_r, x_r, y_r, z_r);

        all_markers[id].vector_pose = v;
        all_markers[id].orientation = q;

        Eigen::Matrix3f R =
            q.normalized().toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix:

        Eigen::Matrix4f p;
        p.setIdentity(); // Set to Identity
        p.block<3, 3>(0, 0) = R;
        p.block<3, 1>(0, 3) = v;

        Eigen::Affine3f aruco_pose;
        aruco_pose.matrix() = p;

        all_markers[id].pose = newPoseOffset(aruco_pose, aruco_close_distance);
    }
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
        transform.setOrigin(tf::Vector3(all_markers[j].pose(0, 3), all_markers[j].pose(1, 3),
                                        all_markers[j].pose(2, 3)));
        transform.setRotation(tf::Quaternion(all_markers[j].w_rotation, all_markers[j].x_rotation,
                                             all_markers[j].y_rotation, all_markers[j].z_rotation));
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
    ros::Subscriber sub = n.subscribe("marker_goal", 1000, listenKeyboardCallback);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, imageCallback);

    ros::spin(); //"while true"
}
