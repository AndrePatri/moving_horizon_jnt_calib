cmake_minimum_required(VERSION 3.16)
project(concert_jnt_calib)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(awesome_utils REQUIRED)

find_package(catkin REQUIRED std_msgs message_generation)

add_service_files(
  FILES
  JntCalibRt.srv
)

add_message_files(
  FILES
  JntCalibStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

add_xbot2_plugin(calib_traj_replayer
    src/calib_traj_replayer.cpp)
target_include_directories(calib_traj_replayer PRIVATE include src ${matlogger2_INCLUDE_DIRS})
add_dependencies(calib_traj_replayer ${PROJECT_NAME}_generate_messages)
target_link_libraries(calib_traj_replayer PRIVATE awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(jnt_calib_rt
    src/jnt_calib_rt.cpp)
target_include_directories(jnt_calib_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} ${base_estimation_INCLUDE_DIRS})
add_dependencies(jnt_calib_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(jnt_calib_rt PRIVATE awesome_utils::awesome_utils xbot2::xbot2_ros_support ${base_estimation_LIBRARIES})

add_xbot2_plugin(iq_model_calib_rt
    src/iq_model_calib_rt.cpp)
target_include_directories(iq_model_calib_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(iq_model_calib_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(iq_model_calib_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(bus_power_rt
    src/bus_power_rt.cpp)
target_include_directories(bus_power_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(bus_power_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(bus_power_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(cart_imp_cntrl_rt
    src/cart_imp_cntrl_rt.cpp)
target_include_directories(cart_imp_cntrl_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(cart_imp_cntrl_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(cart_imp_cntrl_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2 xbot2::xbot2_ros_support)

add_xbot2_plugin(plugins_mngr_rt
    src/plugins_manager_rt.cpp)
target_include_directories(plugins_mngr_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(plugins_mngr_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(plugins_mngr_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(contact_est_rt
    src/contact_est_rt.cpp)
target_include_directories(contact_est_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(contact_est_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(contact_est_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(temp_monitor_rt
    src/temp_monitor_rt.cpp)
target_include_directories(temp_monitor_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(temp_monitor_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(temp_monitor_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(idler_rt
    src/idler_rt.cpp)
target_include_directories(idler_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(idler_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(idler_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(timer_rt
    src/timer_rt.cpp)
target_include_directories(timer_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(timer_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(timer_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

add_xbot2_plugin(impact_detector_rt
    src/impact_detector_rt.cpp)
target_include_directories(impact_detector_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} )
add_dependencies(impact_detector_rt ${PROJECT_NAME}_generate_messages)
target_link_libraries(impact_detector_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support)

if(${behaviortree_cpp_v3_FOUND})

    message("Will compile behaviour tree rt plugin")

    # bt actions/conditions/etc...

    set(BT_CUSTOM_NODES_LIB "bt_custom_nodes")
    add_library(${BT_CUSTOM_NODES_LIB} SHARED
        src/actions/start_plugins.cpp src/actions/stop_plugins.cpp src/actions/set2idle.cpp
        src/actions/restart_jump_sequence.cpp src/actions/perform_takeoff.cpp src/actions/go2takeoff_config.cpp
        src/actions/set2safety_stop.cpp  src/actions/ramp_jnt_imp.cpp

        src/actions/always_running.cpp

        src/conditions/are_plugins_running.cpp src/conditions/are_plugins_closed.cpp src/conditions/temperature_ok.cpp
        src/conditions/is_idle.cpp src/conditions/pause_expired.cpp src/conditions/recov_energy_reached.cpp src/conditions/takeoff_performed.cpp
        src/conditions/takeoff_reached.cpp src/conditions/is_safety_stop.cpp src/conditions/is_joint_device_active.cpp
        src/conditions/jnt_imp_ramped.cpp
        )

    target_include_directories(${BT_CUSTOM_NODES_LIB} PUBLIC src/actions src/conditions ${behaviortree_cpp_v3_INCLUDE_DIRS})
    add_dependencies(${BT_CUSTOM_NODES_LIB} ${PROJECT_NAME}_generate_messages)
    target_link_libraries(${BT_CUSTOM_NODES_LIB} PUBLIC ${behaviortree_cpp_v3_LIBRARIES} xbot2::xbot2 xbot2::xbot2_ros_support)

    install(
        TARGETS ${BT_CUSTOM_NODES_LIB} DESTINATION lib
    )

    add_xbot2_plugin(bt_rt
        src/bt_rt.cpp)
    target_include_directories(bt_rt PRIVATE include src ${matlogger2_INCLUDE_DIRS} ${behaviortree_cpp_v3_INCLUDE_DIRS})
    add_dependencies(bt_rt ${PROJECT_NAME}_generate_messages ${BT_CUSTOM_NODES_LIB})
    target_link_libraries(bt_rt PRIVATE ${cartesian_interface_LIBRARIES} awesome_utils::awesome_utils xbot2::xbot2_ros_support ${behaviortree_cpp_v3_LIBRARIES}
                          ${BT_CUSTOM_NODES_LIB})
    install(
        TARGETS bt_rt DESTINATION lib
    )

endif()

install(
    TARGETS
    contact_est_rt
    jmp_replayer_rt
    base_est_rt
    iq_model_calib_rt
    bus_power_rt
    plugins_mngr_rt
    cart_imp_cntrl_rt
    temp_monitor_rt
    idler_rt
    timer_rt
    impact_detector_rt
    DESTINATION lib
)

#install(TARGETS start_jump_replayer DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})