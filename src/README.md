This works of mapping and localization with aruco markers use two steps. Below is the explanation of how to map.
Mapping and Navigation
------------
This is working in progress of a low-cost autonomous robot. The main idea here is to navigate in a known environment using Aruco marker, All my important places are going to have an Aruco marker, in order to the robot to find the marker and to correct the error after it. For example, if a door is an important place, you can put a marker near the door, and send a goal to that place, so the robot will find the marker and reset pose error(I'm still going to add this, I'll consider the pose given by the aruco marker my real pose, not the odom given by the robot).

Mapping and navigation can't be done at the same time, first, we'll see how to create a map.

Mapping 
------------
Start ros.

```bash
roscore 
```

Bring on the turtlebot.

```bash
roslaunch turtlebot_bringup minimal.launch
```
To move turtlebot.

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

begin mapping, this launch also launchs kinect.

```bash
roslaunch turtlebot_navigation gmapping_demo.launch
```

For visualize your map when mapping open rviz

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

This will be in /catkin_ws/devel/lib/autonomous_robot, this will look for all markers id and poses(optional).

```bash
./catkin_ws/devel/lib/autonomous_robot/marker_finder_saver
```
For save all markers id and poses 

```bash
./catkin_ws/devel/lib/autonomous_robot/marker_goals
```

for save the map when the map is finished, /tmp/my_map will erase everything when rebooting.

```bash
rosrun map_server map_saver -f /tmp/my_map
```

Navigation 
------------
Now that we have our map, let's see how to make turtlebot autonomous navigate..
Close all process before(but roscore)/

Open your saved map with amcl.demo.

```bash
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/tmp/my_map.yaml
```

Visualize your map and turtlebot, with rviz, in this step you need to specify where turtlebot is in your map 
and where is the "looking for", for this in rviz you need to use the tool "2d pose estimation".

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

We'll need this to send where we want the turtlebot to go, you can load all markers saved before.

```bash
./catkin_ws/devel/lib/autonomous_robot/autonomous_robot
```

Now, if you want to send turtlebot to a marker

```bash
./catkin_ws/devel/lib/autonomous_robot/marker_goal
"valid id"
```

But if you want to just send turtlebot to a specify point use

```bash
./catkin_ws/devel/lib/autonomous_robot/goal
"waypoint files"
```

All programs explanation 
------------

- ConfigFile.yaml 

Some programs need this file to load their params, I'm going to give a brief explanation about those params.
  - camera_calibration_file : It's the camera calibration yaml, it's by default inside ros_autonomous_robot folder and uses the default params of "kinect, kinect_default.yaml".
  - rgb_topic : rgb topic of ROS, by default it is "camera/rgb/image_raw".
  - depth_topic : depth topic of ROS, by default it is "camera/depth/image_raw".
  - aruco_dic : Aruco dictionary, by default it is "ARUCO".
  - aruco_poses_file : File where the aruco markers poses are going to be saved or loaded, by default it is "aruco _poses".
  - aruco_distance : Minimum distance of the camera and aruco marker valid, this range is used to decrease error, by default it is "4".
  - aruco_marker_size : Aruco marker size, by default it is "0.1778".

You can change any of those parameters the way is better for you.

- Marker Finder Saver needs "camera_calibration_file", "rgb_topic", "aruco_dic", "aruco_poses_file", "aruco_distance", "aruco_marker_size". Marker Finder Saver is going to find any markers inside the range and save it to a file, you must use this while mapping with a 2d algorithym(i.e gmapping) or with a map already computed.
To save all markers to use after, use the program "marker_goal" and type s, this will save the aruco poses inside "aruco_poses_file".
This program is only going to save markers closer than 4m in order to reduce error detection in low-cost cameras. If you want to change the range, change it inside ConfigFile.yaml >> aruco_distance


- Autonomous Robot needs "camera_calibration_file", "rgb_topic", "aruco_dic", "aruco_poses_file", "aruco_marker_size". Autonomous Robot needs to be used with amcl, it will wait for an id marker or a goal to move autonomous and avoiding obstacles, to send the robot to a marker use "marker_goal" and type the number of the marker, and to send the robot to a specific location(x,y) use "goal".


- Motion Estimator ROS needs "rgb_topic", "depth_topic". Motion Estimator ROS will track points and show how the tracked point moved in space.

- Goal will send the robot to a (x, y) position. 

- Marker Goal marker_goal.cpp save all markers id and positions that were saw in any frame when mapping(marker_finder_saver), and while localizing itself(autonomous_robot) sends a goal to move to a marker id.

-Random Goals load a file with x and y positions in the following format:
```bash
3 %Number of waypoints%
x y
x1 y1
x2 y2
```
Beings xs and ys double.

Then makes the robot goes to those positions randomly and saving how many attempts and 
how many succeeds and failures at reaching the positions.

 

