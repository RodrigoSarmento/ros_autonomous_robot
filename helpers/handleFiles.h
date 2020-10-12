#ifndef INCLUDE_HANDLEFILES_H_
#define INCLUDE_HANDLEFILES_H_

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <structures.h>
#include <vector>

class HandleFiles {

public:
    HandleFiles() {}
    /**
     * Save markers in files
     * @param markers as markerFound struct
     * @param aruco_poses_file name of the file to be saved
     */
    void savePoses(std::vector<Pose> poses, std::string aruco_poses_file);
    /**
     * Load markers in files
     * @param aruco_poses_file name of the file to be saved
     * @return MarkerFound structure
    */
    std::vector<Pose> loadPoses(std::string aruco_poses_file, float aruco_close_distance);
};

#endif