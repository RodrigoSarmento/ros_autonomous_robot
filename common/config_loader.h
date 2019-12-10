#ifndef INCLUDE_CONFIG_LOADER_H_
#define INCLUDE_CONFIG_LOADER_H_

#include <cstdio>
#include <cstdlib>
#include <fstream>

using namespace std;

class ConfigLoader{
    public:
        string camera_calibration_file_;
        string rgb_topic_;
        string depth_topic_;
        string aruco_dic_;
        string aruco_poses_file_;
        float aruco_minimum_distance_;
        double aruco_marker_size_;

    void loadParams(string filename);
};

#endif
