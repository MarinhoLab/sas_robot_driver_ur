#pragma once
#include <atomic>
#include <memory>
#include "driver_ur_joint_positions_manager.hpp"
#include "ur_client_library/ur/ur_driver.h"
void handleRobotProgramState(bool program_running);

int communication_thread_loop(std::shared_ptr<urcl::UrDriver> ur_driver,
                              std::shared_ptr<sas::URJointInformationManager> ur_joint_positions_manager,
                              std::atomic_bool* break_loops);
