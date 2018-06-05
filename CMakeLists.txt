cmake_minimum_required(VERSION 2.8.3)

#Compile in c++ 11
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

#Optimization Flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

project(rgbd_rtk)

##ros
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  move_base_msgs
  actionlib

)

##rtk
message(STATUS "Project source directory: " ${PROJECT_SOURCE_DIR})
message(STATUS "Build directory: " ${PROJECT_BINARY_DIR})

#opencv
find_package(VTK 6.2 REQUIRED)
find_package(OpenCV 3.3.1 )
message(STATUS "OpenCV version: " ${OpenCV_VERSION})
#pcl
find_package(PCL 1.8 REQUIRED PATHS "/usr/local/share/pcl-1.8")
message(STATUS "PCL Version " ${PCL_VERSION})
#aruco
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(aruco 2.0 REQUIRED)
if(aruco_FOUND)
	message(STATUS "Aruco found")
	message(STATUS "Aruco INCLUDES: " ${aruco_INCLUDE_DIRS})
	message(STATUS "Aruco LIBS: " ${aruco_LIBS})
else()
	message(FATAL_ERROR "Error: Aruco was not found in your system")
endif()

catkin_package(

)


include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} common io tracking motion_estimation visualization)
link_directories(${PCL_LIBRARY_DIRS})
add_subdirectory(io)
set(RGBD_RTK_IO rgbd_rtk_io)
add_subdirectory(tracking)
set(RGBD_RTK_TRACKING rgbd_rtk_tracking)
add_subdirectory(common)
set(RGBD_RTK_COMMON rgbd_rtk_common)
add_subdirectory(visualization)
set(RGBD_RTK_VISUALIZATION rgbd_rtk_visualization)
add_subdirectory(motion_estimation)
set(RGBD_RTK_MOTION_ESTIMATION rgbd_rtk_motion_estimation)
add_subdirectory(applications)


message(STATUS "CATKIN LIBRARIES " ${OpenCV_LIBS})


