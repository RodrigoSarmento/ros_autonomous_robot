cmake_minimum_required(VERSION 2.8.3)
project(Autonomous_Robot)

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
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )


message(STATUS ">>> src subdirectory")

add_executable(random_goals random_goals.cpp)
add_executable(marker_goal marker_goal.cpp)
add_executable(autonomous_robot autonomous_robot.cpp)
add_executable(marker_finder_saver marker_finder_saver.cpp)
add_executable(motion_estimator_ros motion_estimator_ros.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)


target_link_libraries(random_goals  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(marker_goal  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(autonomous_robot  ${PCL_LIBRARIES} ${RGBD_RTK_LIBRARIES} ${catkin_LIBRARIES}
                                         ${OpenCV_LIBS} ${aruco_LIBS})
target_link_libraries(marker_finder_saver  ${PCL_LIBRARIES} ${RGBD_RTK_LIBRARIES} ${catkin_LIBRARIES}
                                            ${OpenCV_LIBS} ${aruco_LIBS})
target_link_libraries(motion_estimator_ros  ${PCL_LIBRARIES} ${RGBD_RTK_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} )


install(TARGETS   
		random_goals
		marker_goal
		autonomous_robot		
		marker_finder_saver
		motion_estimator_ros
        RUNTIME DESTINATION bin)
