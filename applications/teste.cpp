#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(){
    string filename = "../../../src/ros_autonomous_robot/ConfigFile.yaml";
    FileStorage fs(filename, FileStorage::READ);
    fs.open(filename, FileStorage::READ);
    string rola;

    fs["aruco_poses_file"] >> rola;
    cout<<rola;
    fs.release();
}