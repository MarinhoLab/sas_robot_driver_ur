#pragma once
/*
# Copyright (c) 2016-2024 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver_kuka.
#
#    sas_robot_driver_kuka is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_kuka is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_kuka.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#   Based on sas_robot_driver_denso.h
#
# ################################################################*/

#include <atomic>
#include <thread>

#include <sas_core/sas_robot_driver.hpp>

using namespace Eigen;

namespace sas
{
//Declared internally
class DriverBcap;

struct RobotDriverURConfiguration
{
    // Examples from UR
    //const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
    //const std::string SCRIPT_FILE = "resources/external_control.urscript";
    //const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
    //const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";
    //const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

    std::string ip;
    std::string script_file;
    std::string output_recipe;
    std::string input_recipe;
    std::string calibration_checksum;
    std::tuple<VectorXd,VectorXd> joint_limits;
};

class RobotDriverUR: public RobotDriver
{
private:
    RobotDriverURConfiguration configuration_;
    std::thread fri_thread_;

    //Implementation details that depend on FRI source files.
    class Impl;
    std::unique_ptr<Impl> impl_;
public:

    RobotDriverUR(const RobotDriverUR&)=delete;
    RobotDriverUR()=delete;
    ~RobotDriverUR();

    RobotDriverUR(const RobotDriverURConfiguration &configuration, std::atomic_bool* break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    VectorXd get_joint_velocities() override;
    //void set_target_joint_velocities(const VectorXd& desired_joint_velocities_rads) override; //Not possible (yet?)

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

};
}
