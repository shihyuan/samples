cmake_minimum_required(VERSION 2.8.3)
project(acl_msgs)

find_package(catkin REQUIRED COMPONENTS 
    message_generation 
    std_msgs 
    geometry_msgs
    nav_msgs
    std_srvs
    actionlib_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  QuadGoal.msg
  VehicleList.msg
  ViconState.msg
  QuadWaypoint.msg
  QuadWaypointError.msg
  )

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs actionlib_msgs std_srvs)
