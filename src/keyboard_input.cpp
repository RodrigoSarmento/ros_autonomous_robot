#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <cstdio>
#include <cstdlib>

#include <sstream>

using namespace std;

/**
 * Send string in ROS system that are going to be listen in other programs
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_goal_node"); //initializing ROS
  ros::NodeHandle n;
  ros::Publisher marker_goal_pub = n.advertise<std_msgs::String>("marker_goal", 1000); //Publishing string msg
  ros::Rate loop_rate(10);
  string id_marker;

  while (ros::ok())
  {
  
    std_msgs::String msg;
    std::stringstream ss;

    printf("To send a goal to a marker ID use : (E.g ID 55) '55'(no quotation markes)  \n");
    printf("To save all markers use 's'(no quotation markers)  \n");
    printf("To exit use 'q'(no quotation markers   ");
    getline(cin, id_marker);
    ss << id_marker;
    msg.data = ss.str();

    ROS_INFO("%s\n", msg.data.c_str());

    
    marker_goal_pub.publish(msg); //publishing string typed in ros msg

    ros::spinOnce();

    loop_rate.sleep();

    if(id_marker.compare("q") == 0)//quiting program when 'q' is typed
      break;
  }

  return 0;
}