# Natalnet/LPR RGB-D Reconstruction Toolkit


Introduction
------------

A set of utility code built by the Natalnet/LPR group for 3D reconstruction
applications using RGB-D (e.g. Microsoft Kinect) cameras.

Requirements
------------

- OpenCV: www.opencv.org  (version >= 3.X)
- PCL: www.pointclouds.org (version 1.8)
- ROS: http://www.ros.org (version Kinetic)
- Aruco: https://sourceforge.net/projects/aruco/files/OldVersions/(version 2.0.14)

It is recommended to install both dependencies from source. The following two links will redirect you to their official tutorials to do so.

> [Install OpenCV on Linux](https://docs.opencv.org/3.3.1/d7/d9f/tutorial_linux_install.html#linux-installation])

> [Compiling PCL from source on Linux](http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php)

>[Install ROS on Linux](http://wiki.ros.org/kinetic/Installation)

>[Create a catkin workspace] >> (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Building
------------

- Create a new path in src and copy everything to this path.
- In path "CMAKE" change file "Findaruco.cmake" to your file Findaruco.cmake, this file can be found in your building Aruco path
- In path catkin_ws 

```bash
catkin_make
```

License
------------

This code is distributed under the terms of the [BSD License](https://github.com/natalnet-lpr/rgbd_rtk/blob/master/LICENSE).


Authors
------------

Natalnet Laboratory for Perceptual Robotics, Federal University of Rio Grande do Norte, Brazil.

Contact: rodrigosarmento rodrigosarmentox@gmail.com
Contact: bruno.silva AT ect.ufrn.br


Contact: rodrigosarmento rodrigosarmentox@gmail.com
Contact: bruno.silva AT ect.ufrn.br

