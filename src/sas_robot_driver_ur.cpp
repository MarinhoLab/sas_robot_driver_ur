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
#   Based on sas_robot_driver_kuka.cpp
#
# ################################################################*/


#include "sas_robot_driver_ur/sas_robot_driver_ur.hpp"
#include <iostream>
#include <memory>

#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>

#include <sas_core/eigen3_std_conversions.hpp>

#include "driver_ur_joint_positions_manager.hpp"
#include "driver_ur.hpp"

namespace sas
{

class RobotDriverUR::Impl
{

public:
    std::shared_ptr<urcl::UrDriver> ur_driver_;
    std::unique_ptr<urcl::DashboardClient> dashboard_client_;
    std::shared_ptr<sas::URJointPositionsManager> ur_joint_positions_manager_;

    Impl()
        {

        };



};

RobotDriverUR::RobotDriverUR(const RobotDriverURConfiguration& configuration, std::atomic_bool* break_loops):
    RobotDriver(break_loops),
    configuration_(configuration)
{
    impl_ = std::make_unique<RobotDriverUR::Impl>();
    impl_->dashboard_client_ = std::make_unique<urcl::DashboardClient>(configuration_.ip);
    joint_limits_ = configuration.joint_limits;
}

RobotDriverUR::~RobotDriverUR()
{

}

VectorXd RobotDriverUR::get_joint_positions()
{
    //The UR libraries use std::array and it's more efficient to keep conversion functions out of the realtime loop, hence, here.
    auto std_array = impl_->ur_joint_positions_manager_->get_current_joint_positions();
    std::vector<double> std_vector(std_array.begin(), std_array.end());
    return sas::std_vector_double_to_vectorxd(std_vector);
}

void RobotDriverUR::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    if(desired_joint_positions_rad.size() != 6)
        throw std::runtime_error("Incorrect vector size in RobotDriverUR::set_target_joint_positions");

    urcl::vector6d_t std_array{desired_joint_positions_rad(0),
                               desired_joint_positions_rad(1),
                               desired_joint_positions_rad(2),
                               desired_joint_positions_rad(3),
                               desired_joint_positions_rad(4),
                               desired_joint_positions_rad(5),};
    impl_->ur_joint_positions_manager_->set_target_joint_positions(std_array);
}



void RobotDriverUR::connect()
{
    // Making the robot ready for the program by:
    // Connect the the robot Dashboard
    if (!impl_->dashboard_client_->connect())
        throw std::runtime_error("Could not connect to dashboard");

    // Stop program, if there is one running
    if (!impl_->dashboard_client_->commandStop())
        throw std::runtime_error("Could not send stop program command");

    // Power it off
    if (!impl_->dashboard_client_->commandPowerOff())
        throw std::runtime_error("Could not send Power off command");
}


void RobotDriverUR::initialize()
{
    // Power it on
    if (!impl_->dashboard_client_->commandPowerOn())
        throw std::runtime_error("Could not send Power on command");

    // Release the brakes
    if (!impl_->dashboard_client_->commandBrakeRelease())
        throw std::runtime_error("Could not send BrakeRelease command");

    // Now the robot is ready to receive a program
    std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
    const bool HEADLESS = true;

    auto ur_driver = std::make_shared<urcl::UrDriver>(
        configuration_.ip,
        configuration_.script_file,
        configuration_.output_recipe,
        configuration_.input_recipe,
        &handleRobotProgramState,
        HEADLESS,
        std::move(tool_comm_setup),
        configuration_.calibration_checksum
        );

    fri_thread_ = std::thread(communication_thread_loop, impl_->ur_driver_, impl_->ur_joint_positions_manager_, break_loops_);

    //Set the communication thread to be realtime with SCHED_FIFO.
    sched_param sch;
    int policy;
    pthread_getschedparam(fri_thread_.native_handle(), &policy, &sch);
    sch.sched_priority = 20;
    if (pthread_setschedparam(fri_thread_.native_handle(), SCHED_FIFO, &sch)) {
        std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }

    // We need meaningful information to be obtained from the robot before moving on.
    // In addition, we guarantee that this doesn't lock us with break_loops.
    while (!(*break_loops_) && !impl_->ur_joint_positions_manager_->is_current_joint_position_valid())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
}


/**
 * @brief RobotDriverUR::deinitialize.
 * For safety reasons, this does not throw exceptions.
 */
void RobotDriverUR::deinitialize()
{
    // Stop program, if there is one running
    impl_->dashboard_client_->commandStop();
    // Power it off
    impl_->dashboard_client_->commandPowerOff();

    //To force the thread to shutdown if it hasn't already done so
    *break_loops_ = true;

    if (fri_thread_.joinable())
        fri_thread_.join();
}

void RobotDriverUR::disconnect()
{
    //Disconnect
    impl_->dashboard_client_->disconnect();
}

}
