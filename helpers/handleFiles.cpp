#include <handleFiles.h>

using namespace std;

void HandleFiles::savePoses(Pose poses[255], std::string aruco_poses_file) {
    ofstream arq;
    arq.open(aruco_poses_file);
    int cont = 0;
    for (int k = 0; k <= 255 - 1; k++) {
        if (poses[k].id == 0) continue;
        cont++;
    } // writing in the first line the number of poses that were found

    arq << cont << endl;
    for (int k = 0; k <= 255 - 1; k++) {
        if (poses[k].id == 0) continue;
        arq << poses[k].id << " " << poses[k].x_pose << " " << poses[k].y_pose << " "
            << poses[k].z_pose << " " << poses[k].x_rotation << " " << poses[k].y_rotation << " "
            << poses[k].z_rotation << " " << poses[k].w_rotation << endl;
    }
}
/**
vector<Pose> HandleFiles::loadPoses(string aruco_poses_file) {
    vector<Pose> poses;
    ifstream load_file;
    load_file.open(aruco_poses_file);
    if (!load_file) {
        printf("Unable to open file, check if the file exists");
        exit(1);
    }
    int size = 0;
    // fill waypoints list
    load_file >> size;
    for (int i = 0; i < size; i++) {
        load_file >> poses[i].id;
        load_file >> poses[i].x_pose;
        load_file >> poses[i].y_pose;
        load_file >> poses[i].z_pose;
        load_file >> poses[i].x_rotation;
        load_file >> poses[i].y_rotation;
        load_file >> poses[i].z_rotation;
        load_file >> poses[i].w_rotation;
        printf("Marker number %i -> x: %f y: %f z: %f r_x: %f r_y: %f r_z: %f r_w: %f\n",
               poses[i].id, poses[i].x_pose, poses[i].y_pose, poses[i].z_pose, poses[i].x_rotation,
               poses[i].y_rotation, poses[i].z_rotation, poses[i].w_rotation);
    }
    load_file.close();
}
*/
