#include "driver_ur_joint_positions_manager.hpp"

void sas::URJointInformationManager::set_current_joint_positions(const urcl::vector6d_t& joint_positions)
{
    std::scoped_lock lock(mutex_current_joint_positions_);
    if(!is_current_joint_position_valid())
        current_joint_position_valid_ = true;
    current_joint_positions_ = joint_positions;
}

urcl::vector6d_t sas::URJointInformationManager::get_current_joint_positions()
{
    std::scoped_lock lock(mutex_current_joint_positions_);
    return current_joint_positions_;
}

void sas::URJointInformationManager::set_target_joint_positions(const urcl::vector6d_t &joint_positions)
{
    std::scoped_lock lock(mutex_target_joint_positions_);
    if(!is_target_joint_position_valid())
        target_joint_position_valid_ = true;
    target_joint_positions_ = joint_positions;
}

urcl::vector6d_t sas::URJointInformationManager::get_target_joint_positions()
{
    std::scoped_lock lock(mutex_target_joint_positions_);
    return target_joint_positions_;
}

void sas::URJointInformationManager::set_current_joint_velocities(const urcl::vector6d_t& joint_velocities)
{
    std::scoped_lock lock(mutex_current_joint_velocities_);
    if(!is_current_joint_velocity_valid())
        current_joint_velocity_valid_ = true;
    current_joint_velocities_ = joint_velocities;
}

urcl::vector6d_t sas::URJointInformationManager::get_current_joint_velocities()
{
    std::scoped_lock lock(mutex_current_joint_velocities_);
    return current_joint_velocities_;
}

void sas::URJointInformationManager::set_current_tcp_force(const urcl::vector6d_t& tcp_force)
{
    std::scoped_lock lock(mutex_current_tcp_force_);
    if(!is_current_tcp_force_valid())
        current_tcp_force_valid_ = true;
    current_tcp_force_ = tcp_force;
}

urcl::vector6d_t sas::URJointInformationManager::get_current_tcp_force()
{
    std::scoped_lock lock(mutex_current_tcp_force_);
    return current_tcp_force_;
}
