cmake_minimum_required(VERSION 2.8.3)
project(control_command_selector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostate_machine
  usv_control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rostate_machine usv_control_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_command_selector_node src/control_command_selector.cpp src/control_command_selector_node.cpp)
target_link_libraries(control_command_selector_node ${catkin_LIBRARIES})

# install executables/libs
install(TARGETS control_command_selector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install headers
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)