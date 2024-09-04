/*
# Copyright (c) 2024 Murilo Marques Marinho
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
#
# ################################################################*/
#pragma once
#include <ur_client_library/types.h>
#include <mutex>

namespace sas
{
class URJointInformationManager
{
private:
    bool current_joint_position_valid_{false};
    urcl::vector6d_t current_joint_positions_;
    std::mutex mutex_current_joint_positions_;

    bool target_joint_position_valid_{false};
    urcl::vector6d_t target_joint_positions_;
    std::mutex mutex_target_joint_positions_;

    bool current_joint_velocity_valid_{false};
    urcl::vector6d_t current_joint_velocities_;
    std::mutex mutex_current_joint_velocities_;
public:
    void set_current_joint_positions(const urcl::vector6d_t& joint_positions);
    urcl::vector6d_t get_current_joint_positions();

    void set_target_joint_positions(const urcl::vector6d_t& joint_positions);
    urcl::vector6d_t get_target_joint_positions();

    void set_current_joint_velocities(const urcl::vector6d_t& joint_positions);
    urcl::vector6d_t get_current_joint_velocities();

    bool is_current_joint_position_valid()
    {
        return current_joint_position_valid_;
    }

    bool is_target_joint_position_valid()
    {
        return target_joint_position_valid_;
    }

    bool is_current_joint_velocity_valid()
    {
        return current_joint_velocity_valid_;
    }
};
}
