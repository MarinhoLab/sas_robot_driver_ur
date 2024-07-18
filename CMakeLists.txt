cmake_minimum_required(VERSION 3.8)
project(sas_robot_driver_ur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_robot_driver REQUIRED)

####################################
# CPP Shared Library Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# The most common use case is to merge everything you need to export
# into the same shared library called ${PROJECT_NAME}.
add_library(${PROJECT_NAME} SHARED
    src/sas_robot_driver_ur.cpp
    src/driver_ur.cpp
    src/driver_ur_joint_positions_manager.cpp

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/default_log_handler.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/log.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/helpers.cpp>

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/calibration_checker.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/version_information.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/ur_driver.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/dashboard_client.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/robot_receive_timeout.cpp>

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/primary_package.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_message.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_state.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_message/version_message.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_state/kinematics_info.cpp>

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/rtde_writer.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/rtde_client.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/rtde_package.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/data_package.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/request_protocol_version.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/control_package_start.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/control_package_pause.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/control_package_setup_inputs.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/control_package_setup_outputs.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/get_urcontrol_version.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/text_message.cpp>

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/control/script_command_interface.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/control/script_sender.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/control/reverse_interface.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/control/trajectory_point_interface.cpp>

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/comm/tcp_server.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/comm/tcp_socket.cpp>
    )

ament_target_dependencies(${PROJECT_NAME}
    sas_core
    Eigen3

    )

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<INSTALL_INTERFACE:include>)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/include>

    )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    Eigen3
    sas_core


    )

install(
    DIRECTORY include/
    DESTINATION include

    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include

    )
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Shared Library Block [END] #
##################################

############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
set(RCLCPP_LOCAL_BINARY_NAME sas_robot_driver_ur_node)

add_executable(${RCLCPP_LOCAL_BINARY_NAME}
    src/sas_robot_driver_ur_node.cpp

    )

ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME}
    rclcpp
    sas_common
    sas_core
    sas_robot_driver

)

target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}
    ${PROJECT_NAME}

    )

target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC c_std_99 cxx_std_17)

install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
    DESTINATION lib/${PROJECT_NAME})

unset(RCLCPP_LOCAL_BINARY_NAME)
# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

# According to https://github.com/SmartArmStack/sas_robot_driver/blob/ros2/CMakeLists.txt
##### LAUNCH FILES #####

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

##END## LAUNCH FILES #####

# According to https://github.com/SmartArmStack/sas_datalogger/blob/78ac681f9cb049c5b9715d215f163565f16b5cdc/CMakeLists.txt
##### PYTHON EXECUTABLES #####
install(PROGRAMS
  scripts/joint_interface_example.py
  DESTINATION lib/${PROJECT_NAME}
)
##END## PYTHON EXECUTABLES #####

# According to this file, this is the first time I've attempted this
##### Relevant Resource Files #####
install(FILES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/resources/external_control.urscript>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/examples/resources/rtde_input_recipe.txt>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/examples/resources/rtde_output_recipe.txt>
  DESTINATION share/${PROJECT_NAME}
)
##END## PYTHON EXECUTABLES #####

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()    
