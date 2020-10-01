#include "goal.h"

using namespace std;

bool Goal::send2dGoal(const double &x, const double &y, const Eigen::Quaterniond &orientation) {
    // Define a client for to send goal requests to the move_base server
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // Set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Send goal position
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = orientation.x();
    goal.target_pose.pose.orientation.y = orientation.y();
    goal.target_pose.pose.orientation.z = orientation.z();
    goal.target_pose.pose.orientation.w = orientation.w();

    ROS_INFO("Sending goal location ...");
    ac.sendGoal(goal);

    ac.waitForResult();
    // Goal result
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("You have reached the destination");
        return true;
    } else {
        ROS_INFO("The robot failed to reach the destination");
        return false;
    }
}

bool Goal::send3dGoal(const Eigen::Affine3f &pose, const Eigen::Quaterniond &orientation) {
    // Define a client for to send goal requests to the move_base server
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // Set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Send goal position
    goal.target_pose.pose.position.x = pose(0, 3);
    goal.target_pose.pose.position.y = pose(1, 3);
    goal.target_pose.pose.position.z = pose(2, 3);
    goal.target_pose.pose.orientation.x = orientation.x();
    goal.target_pose.pose.orientation.y = orientation.y();
    goal.target_pose.pose.orientation.z = orientation.z();
    goal.target_pose.pose.orientation.w = orientation.w();

    ROS_INFO("Sending goal location ...");
    ac.sendGoal(goal);

    ac.waitForResult();
    // Goal result
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("You have reached the destination");
        return true;
    } else {
        ROS_INFO("The robot failed to reach the destination");
        return false;
    }
}