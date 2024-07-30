#include "driver_ur_joint_positions_manager.hpp"

void sas::URJointPositionsManager::set_current_joint_positions(const urcl::vector6d_t& joint_positions)
{
    std::scoped_lock lock(mutex_current_joint_positions_);
    if(!is_current_joint_position_valid())
        current_joint_position_valid_ = true;
    current_joint_positions_ = joint_positions;
}

urcl::vector6d_t sas::URJointPositionsManager::get_current_joint_positions()
{
    std::scoped_lock lock(mutex_current_joint_positions_);
    return current_joint_positions_;
}

void sas::URJointPositionsManager::set_target_joint_positions(const urcl::vector6d_t &joint_positions)
{
    std::scoped_lock lock(mutex_target_joint_positions_);
    if(!is_target_joint_position_valid())
        target_joint_position_valid_ = true;
    target_joint_positions_ = joint_positions;
}

urcl::vector6d_t sas::URJointPositionsManager::get_target_joint_positions()
{
    std::scoped_lock lock(mutex_target_joint_positions_);
    return target_joint_positions_;
}
