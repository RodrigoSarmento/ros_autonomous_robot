#ifndef INCLUDE_STRUCTURES_H_
#define INCLUDE_STRUCTURES_H_
#include <Eigen/Dense>
#include <Eigen/Geometry>
struct Pose {
    int id;
    float x;
    float y;
    float z;
    float x_rotation;
    float y_rotation;
    float z_rotation;
    float w_rotation;
};
#endif