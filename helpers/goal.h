#ifndef INCLUDE_GOAL_H_
#define INCLUDE_GOAL_H_

#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <cstdio>
#include <cstdlib>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

class Goal {

public:
    Goal() {}
    /**
     * Sends a 2d goal
     * @param x positions @param y position
     * @param orientation Quaterniond
     * @return true if reached false if didn't
     */
    bool send2dGoal(const double &x, const double &y, const Eigen::Quaterniond &orientation);
    /** Sends a 3d goal
    * @param pose and @param orientation
    * @return true if reached false if didn't
    */
    bool send3dGoal(const Eigen::Affine3f &pose, const Eigen::Quaterniond &orientation);
};

#endif
