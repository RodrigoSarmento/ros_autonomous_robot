cmake_minimum_required(VERSION 2.8.3)

message(STATUS ">>> src subdirectory")
message("LIB AUTONOMOUS_ROBOT_SRC")

add_executable(random_goals random_goals.cpp)
add_executable(keyboard_input keyboard_input.cpp)
add_executable(autonomous_robot autonomous_robot.cpp)
add_executable(marker_finder_saver marker_finder_saver.cpp)
add_executable(motion_estimator_ros motion_estimator_ros.cpp)

find_library(AUTONOMOUS_ROBOT_HELPERS HINTS ${PROJECT_BINARY_DIR}/helpers)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

target_link_libraries(random_goals  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${AUTONOMOUS_ROBOT_HELPERS})
target_link_libraries(keyboard_input  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${AUTONOMOUS_ROBOT_HELPERS})
target_link_libraries(autonomous_robot  ${PCL_LIBRARIES} ${RGBD_RTK_LIBRARIES} ${catkin_LIBRARIES}
                                         ${OpenCV_LIBS} ${aruco_LIBS} ${AUTONOMOUS_ROBOT_HELPERS})
target_link_libraries(marker_finder_saver  ${PCL_LIBRARIES} ${RGBD_RTK_LIBRARIES} ${catkin_LIBRARIES}
                                            ${OpenCV_LIBS} ${aruco_LIBS} ${AUTONOMOUS_ROBOT_HELPERS})
target_link_libraries(motion_estimator_ros  ${PCL_LIBRARIES} ${RGBD_RTK_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} )


install(TARGETS   
		random_goals
		keyboard_input
		autonomous_robot		
		marker_finder_saver
		motion_estimator_ros
        RUNTIME DESTINATION bin)
