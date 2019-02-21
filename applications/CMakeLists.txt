cmake_minimum_required(VERSION 2.6)

#Compile in c++ 11
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

#Optimizations Flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

##

project(RGBD_RTK)

message(STATUS ">>> applications subdirectory")

add_executable(autonomous_robot autonomous_robot.cpp)
add_executable(goal goal.cpp)
add_executable(marker_finder_saver marker_finder_saver.cpp)
add_executable(motion_estimator_ros motion_estimator_ros.cpp)
add_executable(bag_loader bag_loader.cpp)
add_executable(marker_goal marker_goal.cpp)




set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

find_library(RGBD_RTK_IO NAMES rgbd_rtk_io)
find_library(RGBD_RTK_TRACKING NAMES rgbd_rtk_tracking)
find_library(RGBD_RTK_COMMON NAMES rgbd_rtk_common)
find_library(RGBD_RTK_VISUALIZATION NAMES rgbd_rtk_visualization)
find_library(RGBD_RTK_MOTION_ESTIMATION NAMES rgbd_rtk_motion_estimation)


target_link_libraries(autonomous_robot  ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING} ${RGBD_RTK_SLAM}
                                             ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION}
                                             ${RGBD_RTK_MOTION_ESTIMATION} ${catkin_LIBRARIES}
 ${OpenCV_LIBS} ${aruco_LIBS})
target_link_libraries(goal  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(marker_finder_saver  ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
                                             ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION}
                                             ${RGBD_RTK_MOTION_ESTIMATION} ${catkin_LIBRARIES}  ${RGBD_RTK_SLAM}
 ${OpenCV_LIBS} ${aruco_LIBS})
target_link_libraries(motion_estimator_ros  ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
                                             ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION}
                                             ${RGBD_RTK_MOTION_ESTIMATION} ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(bag_loader   ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(marker_goal  ${catkin_LIBRARIES} ${OpenCV_LIBS})




install(TARGETS autonomous_robot		
		goal
		marker_finder_saver
		motion_estimator_ros
		bag_loader
		marker_goal
        RUNTIME DESTINATION bin)
