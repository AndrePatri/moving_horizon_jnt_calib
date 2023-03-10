cmake_minimum_required(VERSION 3.16)
project(moving_horizon_jnt_calib)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(awesome_utils REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(xbot2 REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(matlogger2 REQUIRED)

find_package(catkin REQUIRED roscpp rospy urdf sensor_msgs std_msgs message_generation)

add_service_files(
  FILES
  JntCalibRt.srv
  PerformCalibTraj.srv
)

add_message_files(
  FILES
  JntCalibStatus.msg
  CalibTrajStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

add_xbot2_plugin(calib_traj_replayer_rt
    src/calib_traj_replayer_rt.cpp)
target_include_directories(calib_traj_replayer_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS})
add_dependencies(calib_traj_replayer_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(calib_traj_replayer_rt PRIVATE awesome_utils::awesome_utils xbot2::xbot2_ros_support matlogger2::matlogger2)

#add_xbot2_plugin(jnt_calib_rt
#    src/jnt_cal_rt.cpp)
#target_include_directories(jnt_calib_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} ${base_estimation_INCLUDE_DIRS})
#add_dependencies(jnt_calib_rt ${PROJECT_NAME}_generate_messages)
#target_link_libraries(jnt_calib_rt PRIVATE awesome_utils::awesome_utils xbot2::xbot2_ros_support ${base_estimation_LIBRARIES})

add_executable(calib_logger src/calib_logger.cpp)
target_link_libraries(calib_logger ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES} matlogger2::matlogger2)
target_include_directories(calib_logger PRIVATE ${catkin_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS} ${xbot_msgs_INCLUDE_DIRS})
set_target_properties(calib_logger PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(
    TARGETS
    calib_traj_replayer_rt
#    jnt_calib_rt
    DESTINATION lib
)

install(TARGETS calib_logger DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS src/launch_rosbag.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
