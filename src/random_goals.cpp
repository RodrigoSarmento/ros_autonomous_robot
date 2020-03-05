//C++
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdio.h>
#include <time.h>
//ROS
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


using namespace std;

struct waypoint{ //Structure used to create a list of waypoints, n is the number of waypoints saved
   int n;
   double x;
   double y;
};

// function declarations
bool moveToGoal(double x, double y); //move to a position x y in map
void randomlygoals(waypoint waypoints[], int n); //Main loop that makes the robot goes to randomly positions
void loadWaypoints(waypoint waypoints[], int n, string waypoints_file); //Load waypoints in file
//Variable declarations
waypoint waypoints[200];
int local = 1;
int attempts = 0;
int sucess = 0;
int failed = 0;

int main(int argc, char** argv){
   ros::init(argc, argv, "loop_goals_node"); //initializing node

   if(argc != 2){
      fprintf(stderr, "Usage : %s <waypoints file>\n", argv[0]); //need to give the waypoints file
      exit(0);
   }   

   string waypoints_file;
   waypoints_file = argv[1]; // reading waypoints file
   loadWaypoints(waypoints, 200, waypoints_file);  //loading waypoints to waypoint list

   srand (time(NULL));

   randomlygoals(waypoints,waypoints[0].n); //Main loop
   ros::spinOnce();
   
   return 0;
}

/**
 * Send a goal to the robot
 * @Params X and Y position
 * @Return boolean if the robot sucessed or failed in reach position
 */
bool moveToGoal(double x, double y){
   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   // moving towards the goal

   goal.target_pose.pose.position.x =  x;
   goal.target_pose.pose.position.y =  y;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();
   //If the robot reaches the position, returns true, otherwise returns false
   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) return true;     
   else return false;
}

/**
 * choose a random known position and send a goal 
 * @Params list of waypoints, number of waypoints
 */
void randomlygoals(waypoint waypoints[], int n){
   int i=0;
   int i_last=-1;
   ofstream arq;
   arq.open("attemps.txt");
      while(true){
         i = rand() % waypoints[0].n;//sorting a randomly position
         if(i == i_last){ //making sure that the robot is not trying to go to where he already is
            continue;
         }

         printf("Attempts destination: %i\nReached: %i \nFailed: %i \n\n", attempts,sucess,failed);
         printf("Trying to go to waypoint %i: x = %f y = %f\n",i, waypoints[i].x, waypoints[i].y);

         if(moveToGoal(waypoints[i].x, waypoints[i].y) == true){//If the robot reaches the position update the sucess and attempts
            printf("Robot reached waypoint\n");
            sucess++;
            attempts++;
         }
         else{//If the robot failed to reached the position update the failed and attempts
            printf("Robot faild to reach waypoint\n");
            failed++;
            attempts++;
         }

         if(arq.is_open()){//Save in file the recent update
            arq.seekp(0, ios::beg);
            arq<<"Attempts : "<<attempts<<endl;
            arq<<"Sucess : "<<sucess<<endl;
            arq<<"Failed :"<<failed<<endl;
         }
         else cout<<"file couldn't be open\n";

      i_last = i;
      }
}

/**
 * Load a file of waypoints
 */
void loadWaypoints(waypoint waypoints[], int n, string waypoints_file){
   // open and read file
   ifstream load_file;
   load_file.open(waypoints_file);
   if(!load_file){
      cout << "Unable to open file";
      exit(1);
   }

   // fill waypoints list
   load_file>>n;
   waypoints[0].n=n;
   for(int i = 0; i < n; i++){
      load_file>>waypoints[i].x;
      load_file>>waypoints[i].y;
   }
   load_file.close();
}