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

using namespace std;

struct waypoint { // Structure used to create a list of waypoints, n is the number of waypoints
    int id;
    double x;
    double y;
    double z;
    double r_x;
    double r_y;
    double r_z;
    double r_w;
};

struct attempts { // Structure to save the number of attempts
    int total = 0;
    int success = 0;
    int failed = 0;
    int numberOfWaypoints = 0;
};

bool moveToGoal(double x, double y, double r_x, double r_y, double r_z, double r_w);
void randomlyGoals(Goal goal);
void loadWaypoints(string waypoints_file);

// Variable declarations
waypoint waypoints[200];
attempts attempts;

int main(int argc, char **argv) {
    ros::init(argc, argv, "loop_goals_node"); // Initializing node

    if (argc != 2) {
        fprintf(stderr, "Usage : %s <waypoints file>\n", argv[0]); // Need to give the waypoints
        exit(0);
    }
    Goal goal;

    // Reading waypoints
    string waypoints_file;
    waypoints_file = argv[1];
    loadWaypoints(waypoints_file);

    // Main loop
    randomlyGoals(goal);
    ros::spinOnce();

    return 0;
}

/**
 * choose a random known position and send a goal
 */
void randomlyGoals(Goal goal) {
    srand(time(NULL));

    int i = 0;
    int i_last = -1;
    ofstream arq;
    arq.open("attemps.txt");

    while (true) {
        i = rand() % attempts.numberOfWaypoints; // sorting a randomly position
        // making sure that the robot is not trying to go to where he already is
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

/**
 * Load a file of waypoints and save it in waypoints structure
 */
void loadWaypoints(string waypoints_file) {
    // open and read file
    ifstream load_file;
    load_file.open(waypoints_file);
    if (!load_file) {
        cout << "Unable to open file, check if the file exists";
        exit(1);
    }

    // fill waypoints list
    load_file >> attempts.numberOfWaypoints;
    for (int i = 0; i < attempts.numberOfWaypoints; i++) {
        load_file >> waypoints[i].id;
        load_file >> waypoints[i].x;
        load_file >> waypoints[i].y;
        load_file >> waypoints[i].z;
        load_file >> waypoints[i].r_x;
        load_file >> waypoints[i].r_y;
        load_file >> waypoints[i].r_z;
        load_file >> waypoints[i].r_w;
        printf("Waypoint number %i -> x: %f y: %f z: %f r_x: %f r_y: %f r_z: %f r_w: %f\n",
               waypoints[i].id, waypoints[i].x, waypoints[i].y, waypoints[i].z, waypoints[i].r_x,
               waypoints[i].r_y, waypoints[i].r_z, waypoints[i].r_w);
    }
    load_file.close();
}