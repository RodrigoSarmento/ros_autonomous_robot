#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <cstdio>
#include <cstdlib>

#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker"); //initializing ROS
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000); //Publishing string msg
  ros::Rate loop_rate(10);
  string id_marker;
  cout<<"q to quit\n";

  while (ros::ok())
  {
  
    std_msgs::String msg;
    std::stringstream ss;

    printf("For send a goal to a marker ID use : (E.g ID 55) '55'(no quotation markes)\n");
    printf("For save all markers use 's'(no quotation markers)\n");
    printf("For exit use 'q'(no quotation markers");
    cout<<"Insira a ID\n";
    getline(cin, id_marker);
    ss << id_marker;
    msg.data = ss.str();

    ROS_INFO("%s\n\n\n\n\n", msg.data.c_str());

    
    chatter_pub.publish(msg); //publishing string typed in ros msg

    ros::spinOnce();

    loop_rate.sleep();

    if(id_marker.compare("q") == 0)//quiting program when 'q' is typed
      break;

  }


  return 0;
}