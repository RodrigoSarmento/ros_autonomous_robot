cmake_minimum_required(VERSION 2.6)

message(STATUS ">>> helpers subdirectory")

set(AUTONOMOUS_ROBOT_HELPERS_SRC goal.cpp handleFiles.cpp)
set(AUTONOMOUS_ROBOT_HELPERS_INCLUDES goal.h handleFiles.h structures.h)

add_library(autonomous_robot_helpers SHARED ${AUTONOMOUS_ROBOT_HELPERS_SRC})
target_link_libraries(autonomous_robot_helpers ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})

set_target_properties(autonomous_robot_helpers PROPERTIES OUTPUT_NAME "autonomous_robot_helpers")
set_target_properties(autonomous_robot_helpers PROPERTIES VERSION 1.0)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

install(TARGETS autonomous_robot_helpers
  		  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  		  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  		  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/autonomous_robot_helpers"
    	  COMPONENT dev)
install(FILES ${AUTONOMOUS_ROBOT_HELPERS_INCLUDES} DESTINATION include)