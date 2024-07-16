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

namespace sas
{

class RobotDriverUR::Impl
{
public:
    std::shared_ptr<urcl::UrDriver> ur_driver_;
    std::unique_ptr<urcl::DashboardClient> dashboard_client_;

    Impl()
        {

        };

};

RobotDriverUR::RobotDriverUR(const RobotDriverURConfiguration& configuration, std::atomic_bool* break_loops):
    RobotDriver(break_loops),
    configuration_(configuration)
{
    impl_ = std::make_unique<RobotDriverUR::Impl>();
    joint_limits_ = configuration.joint_limits;
}

RobotDriverUR::~RobotDriverUR()
{

}

VectorXd RobotDriverUR::get_joint_positions()
{
    return impl_->trafo_client_->get_measured_joint_values();
}

void RobotDriverUR::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    impl_->trafo_client_->set_target_joint_values(desired_joint_positions_rad);
}

void RobotDriverUR::connect()
{
    std::atomic_bool connection_state(false); //Unknown connection state
    fri_thread_ = std::thread(communication_thread_loop, impl_->trafo_client_, break_loops_, &connection_state);

    //Set the communication thread to be realtime with SCHED_FIFO.
    sched_param sch;
    int policy;
    pthread_getschedparam(fri_thread_.native_handle(), &policy, &sch);
    sch.sched_priority = 20;
    if (pthread_setschedparam(fri_thread_.native_handle(), SCHED_FIFO, &sch)) {
        std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }

    //We need the connection to be established before moving on.
    //However, we guarantee that this doesn't lock us with break_loops.
    while (!(*break_loops_) && !connection_state)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void RobotDriverUR::disconnect()
{
    //To force the thread to shutdown if it hasn't already done so
    *break_loops_ = true;

    if (fri_thread_.joinable())
        fri_thread_.join();
}

void RobotDriverUR::initialize()
{
    impl_->dashboard_client_ = std::make_unique<urcl::DashboardClient>(configuration_.ip);
}

void RobotDriverUR::deinitialize()
{
    //Nothing to do
}

}
