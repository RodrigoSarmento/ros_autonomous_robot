This works of mapping and localization with aruco markers uses two steps. Below is the explanation of how to map.
Mapping and Navigation
------------


Mapping and navigation can't be done at the same time, first we'll see how to create a map.

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

begin mapping,this launch also launchs kinect.

```bash
roslaunch turtlebot_navigation gmapping_demo.launch
```

For vizualize your map when mapping open rviz

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

Vizualize your map and turtlebot, with rviz, in this step you need to specify where turtlebot is in your map 
and where is he "looking for", for this in rviz you need to use the tool "2d pose estimation".

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

We'll need this for send where we want the turtlebot to go, you can load all markers saved before.

```bash
./catkin_ws/devel/lib/autonomous_robot/autonomous_robot
```

Now, if you want to send turtlebot to an marker

```bash
./catkin_ws/devel/lib/autonomous_robot/marker_goal
"valid id"
```

But if you want to just send turtlebot to an specify point use

```bash
./catkin_ws/devel/lib/autonomous_robot/goal
"waypoint files"
```

All programs explanation 
------------

- Marker Finder Saver

You can change the ARUCO marker dict in >> slam>> marker_finder.cpp >> "	marker_detector_.setDictionary("ARUCO_MIP_36h12", 0);"
marker_finder_saver.cpp will find markers and save it id and position, it needs camera parameters, markers size, 'marker_saver.txt' and a ros rgb topic.
To save all markers for use after, use the program "marker_goal" and type s.
This program is only going to save markers closer than 4m in order to resuce error detection in low cost cameras. If you want to change the range, you can do it in marker_finder.cpp "setMarkerPointPosesGlobal" functino

by default it will use "camera/rgb/image_raw", if you want to use an equivalente topic just type when launch the program as "./motion_estimator camera/rgb/image_color

- Autonomous Robot it needs camera parameters, markers size, 'marker_saver.txt'(from marker_finder_saver.cpp). Autonomous_robot.cpp 
will wait for an id marker or a goal to move autonomous and avoiding obstacles, to send the robot to a marker use "marker_goal", and to send the robot to a specificy location(x,y) use "goal".

by default it will use "camera/rgb/image_raw", if you want to use an equivalente topic just type when launch the program as "./motion_estimator camera/rgb/image_color

- Motion estimator ros

motion_estimator.cpp will track points and show how the tracked point moved in space.

Motion estimator needs two ros topics to work rgb and depth topic, by default it will use "camera/rgb/image_raw" and "camera/depth/image_raw",
if you want to use an equivalente topic just type when launch the program as "./motion_estimator camera/rgb/image_color camera/depth/image.

- Bag Loader

bag_loader.cpp will load and show in window a rgb and a depth topic

bag_loader needs two ros topics to work rgb and depth topic, by default it will use "camera/rgb/image_raw" and "camera/depth/image_raw",
if you want to use an equivalente topic just type when launch the program as "./bag_loader camera/rgb/image_color camera/depth/image.

- Goal

goal.cpp will send the robot to all waypoints randomly 

- marker_goal

marker_goal.cpp save all markers id and positions that were saw in any frame when mapping(marker_finder_saver), and while localizating itself(autonomous_robot) sends a goal to move to a marker id.

-Random Goals

random_goals.cpp load a file with x and y positions in the following format:
```bash
3 %Number of waypoints%
x y
x1 y1
x2 y2
```
Beings xs and ys double.

Than makes the robot goes to those positions randomly and saving how many attempts and 
how many succeds and failures at reaching the positions.

 

