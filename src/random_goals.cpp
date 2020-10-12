// C++
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdio.h>
#include <time.h>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <goal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include <handleFiles.h>

using namespace std;

struct attempts { // Structure to save the number of attempts
    int total = 0;
    int success = 0;
    int failed = 0;
};

void randomlyGoals();
void loadWaypoints(string waypoints_file);

// Variable declarations
vector<Pose> waypoints;
attempts attempts;
HandleFiles handleFiles;

int main(int argc, char **argv) {
    ros::init(argc, argv, "loop_goals_node"); // Initializing node

    if (argc != 2) {
        fprintf(stderr, "Usage : %s <waypoints file>\n", argv[0]); // Need to give the waypoints
        exit(0);
    }

    // Reading waypoints
    string waypoints_file;
    waypoints_file = argv[1];
    waypoints = handleFiles.loadPoses(waypoints_file, 0.1); // loading markers

    // Main loop
    randomlyGoals();
    ros::spinOnce();

    return 0;
}

/**
 * Choose a random known position and send a goal
 */
void randomlyGoals() {
    Goal goal;
    srand(time(NULL));

    int i = 0;
    int i_last = -1;
    ofstream arq;
    arq.open("attemps.txt");

    while (true) {
        i = rand() % waypoints.size(); // Sorting a randomly position
        // Making sure that the robot is not trying to go to where he already is
        if (i == i_last) continue;
        printf("Attempts destination: %i\nReached: %i \nFailed: %i \n\n", attempts.total,
               attempts.success, attempts.failed);
        printf("Trying to go to waypoint %i: x = %f y = %f\n", i, waypoints[i].x, waypoints[i].y);

        if (goal.send2dGoal(waypoints[i].x, waypoints[i].y, Eigen::Quaterniond(1, 0, 0, 0)) ==
            true) {
            // If the robot reaches the position update the success and attempts
            printf("Robot reached waypoint\n");
            attempts.success++;
            attempts.total++;
        } else { // If the robot failed to reached the position update the failed and attempts
            printf("Robot failed to reach waypoint\n");
            attempts.failed++;
            attempts.total++;
        }

        if (arq.is_open()) { // Save in file the recent update
            arq.seekp(0, ios::beg);
            arq << "Attempts : " << attempts.total << endl;
            arq << "Success : " << attempts.success << endl;
            arq << "Failed :" << attempts.failed << endl;
        } else
            cout << "file couldn't be open\n";

        i_last = i;
    }
}
