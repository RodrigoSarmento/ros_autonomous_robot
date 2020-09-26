#ifndef INCLUDE_STRUCTURES_H_
#define INCLUDE_STRUCTURES_H_

struct Pose {
    int id;
    float x_pose;
    float y_pose;
    float z_pose;
    float x_rotation;
    float y_rotation;
    float z_rotation;
    float w_rotation;
};
#endif