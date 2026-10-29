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
find_package(sas_force_sensor REQUIRED)

# The vendored Universal_Robots_Client_Library generates this header from
# its own (never configured) CMakeLists; we compile its sources directly
# into this package, so generate it here (default matches the upstream
# option PRIMARY_CLIENT_STRICT_PARSING_ OFF).
set(PRIMARY_CLIENT_STRICT_PARSING_ OFF)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/compile_options.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated_include/ur_client_library/compile_options.h
  )

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

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/primary_client.cpp>
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

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/types.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_message/error_code_message.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_message/hardware_info_message.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_message/key_message.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_message/runtime_exception_message.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_message/safety_mode_message.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_message/text_message.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_state/configuration_data.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_state/masterboard_data.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/primary/robot_state/robot_mode_data.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/data_type.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/read_properties.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/robot_properties.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/rtde/rtde_parser.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/control/motion_primitives.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/control/script_reader.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/dashboard_client_implementation_g5.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/dashboard_client_implementation_x.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/error_code_overrides.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/instruction_executor.cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/src/ur/tool_communication.cpp>
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/3rdparty>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated_include>

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
    sas_force_sensor

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

# According to this file, this is the first time I've attempted this
##### Relevant Resource Files #####
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/resources/external_control.urscript
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/examples/resources/rtde_input_recipe.txt
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ur/Universal_Robots_Client_Library/examples/resources/rtde_output_recipe.txt
  DESTINATION share/${PROJECT_NAME}
)
##END## PYTHON EXECUTABLES #####

##### Robot config files #####
install(DIRECTORY
  robots
  DESTINATION share/${PROJECT_NAME}/
)
##END## Robot config files #####

##### Config files #####
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
##END## Config files #####

# According to https://github.com/SmartArmStack/sas_robot_driver/blob/ros2/CMakeLists.txt
##### LAUNCH FILES #####

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

##END## LAUNCH FILES #####

##### Calibration files #####
install(DIRECTORY
  calibration
  DESTINATION share/${PROJECT_NAME}/
)
##END## Calibration files #####

ament_package()    
