/*
# Copyright (c) 2016-2024 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver_ur.
#
#    sas_robot_driver_ur is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_ur is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_ur.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#   Based on sas_robot_driver_kuka
#
# ################################################################*/
#include <rclcpp/rclcpp.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_robot_driver_ur/sas_robot_driver_ur.hpp>
#include <dqrobotics/utils/DQ_Math.h>

using namespace DQ_robotics;

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);

    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_kuka");

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverURConfiguration configuration;

        //configuration.ip = "192.168.1.245";
        //configuration.script_file = "/home/b40617mm/ros2_ws/install/sas_robot_driver_ur/share/sas_robot_driver_ur/external_control.urscript";
        //configuration.output_recipe = "/home/b40617mm/ros2_ws/install/sas_robot_driver_ur/share/sas_robot_driver_ur/rtde_output_recipe.txt";
        //configuration.input_recipe = "/home/b40617mm/ros2_ws/install/sas_robot_driver_ur/share/sas_robot_driver_ur/rtde_input_recipe.txt";
        //configuration.calibration_checksum = "calib_12788084448423163542";
        sas::get_ros_parameter(node,"ip",configuration.ip);
        sas::get_ros_parameter(node,"script_file",configuration.script_file);
        sas::get_ros_parameter(node,"output_recipe",configuration.output_recipe);
        sas::get_ros_parameter(node,"input_recipe",configuration.input_recipe);
        sas::get_ros_parameter(node,"calibration_checksum",configuration.calibration_checksum);

        //std::vector<double> joint_limits_min{-360.0, -360.0, -360.0, -360.0, -360.0, -720.0};
        //std::vector<double> joint_limits_max{360.0, 360.0, 360.0, 360.0, 360.0, 720.0};
        std::vector<double> joint_limits_min;
        std::vector<double> joint_limits_max;
        sas::get_ros_parameter(node,"joint_limits_min",joint_limits_min);
        sas::get_ros_parameter(node,"joint_limits_max",joint_limits_max);
        configuration.joint_limits = {deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_min)),
                                      deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_max))};

        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        //robot_driver_ros_configuration.thread_sampling_time_sec = 0.001;
        sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");


        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverKuka.");
        auto robot_driver_ur = std::make_shared<sas::RobotDriverUR>(configuration,
                                                                        &kill_this_process);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_ur,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    return 0;
}
