cmake_minimum_required(VERSION 2.8.3)
project(motor)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  message_generation
)


add_message_files(
  FILES
  Flow.msg
  # Message2.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
   CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES px4flow
#  CATKIN_DEPENDS roscpp serial std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(swarm_serial_send src/robot_swarm_serial_send.cpp)
target_link_libraries(swarm_serial_send ${catkin_LIBRARIES})

add_executable(Posi_Vel_pubandsub src/Posi_Vel_pubandsub.cpp)
target_link_libraries(Posi_Vel_pubandsub ${catkin_LIBRARIES})
