#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry.h>
#include <handleFiles.h>

using namespace std;

void HandleFiles::savePoses(vector<Pose> poses, std::string aruco_poses_file) {
    ofstream arq;
    arq.open(aruco_poses_file);

    for (Pose pose : poses) {
        arq << pose.id << " " << pose.x << " " << pose.y << " " << pose.z << " " << pose.x_rotation
            << " " << pose.y_rotation << " " << pose.z_rotation << " " << pose.w_rotation << endl;
    }
}

vector<Pose> HandleFiles::loadPoses(string aruco_poses_file, float aruco_close_distance = 0.0) {
    vector<Pose> poses;
    Pose pose;
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
        load_file >> pose.id;
        load_file >> pose.x;
        load_file >> pose.y;
        load_file >> pose.z;
        load_file >> pose.x_rotation;
        load_file >> pose.y_rotation;
        load_file >> pose.z_rotation;
        load_file >> pose.w_rotation;
        printf("Marker number %i -> x: %f y: %f z: %f r_x: %f r_y: %f r_z: %f r_w: %f\n",
               poses[i].id, poses[i].x, poses[i].y, poses[i].z, poses[i].x_rotation,
               poses[i].y_rotation, poses[i].z_rotation, poses[i].w_rotation);

        Eigen::Vector3f v(pose.x, pose.y, pose.z);
        Eigen::Quaternionf q(pose.w_rotation, pose.x_rotation, pose.y_rotation, pose.z_rotation);

        pose.vector_pose = v;
        pose.quaternion_orientation = q;

        Eigen::Matrix3f R = q.normalized().toRotationMatrix();
        Eigen::Matrix4f p;
        p.setIdentity();

        p.block<3, 3>(0, 0) = R;
        p.block<3, 1>(0, 3) = v;

        Eigen::Affine3f aruco_pose;
        aruco_pose.matrix() = p;

        pose.affine_pose = newPoseOffset(aruco_pose, aruco_close_distance);
        poses.push_back(pose);
    }
    load_file.close();
    return poses;
}
