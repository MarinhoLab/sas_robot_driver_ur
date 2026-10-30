/*
# Copyright (c) 2012-2026 Murilo Marques Marinho
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
*/

#pragma once
#include <atomic>
#include <memory>
#include "driver_ur_joint_positions_manager.hpp"
#include "ur_client_library/ur/ur_driver.h"
void handleRobotProgramState(bool program_running);

int communication_thread_loop(std::shared_ptr<urcl::UrDriver> ur_driver,
                              std::shared_ptr<sas::URJointInformationManager> ur_joint_positions_manager,
                              std::atomic_bool* break_loops);
