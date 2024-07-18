//Heavily modified from examples/full_driver.cpp, see original license below.
//2024, Murilo M. Marinho, (www.murilomarinho.info)
// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-08-06
 */
//----------------------------------------------------------------------

#include <iostream>
#include <memory>

#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include "driver_ur_joint_positions_manager.hpp"

using namespace urcl;

// We need a callback function to register. See UrDriver's parameters for details.
void handleRobotProgramState(bool program_running)
{
    // Print the text in green so we see it better
    std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

int communication_thread_loop(std::shared_ptr<UrDriver> ur_driver,
                              std::shared_ptr<sas::URJointPositionsManager> ur_joint_positions_manager,
                              std::atomic_bool* break_loops)
{
    //Murilo: despite the many modifications, I've left the original comments from the example, hoping that they might be useful in the future.

    // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
    // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
    // loop.
    ur_driver->startRTDECommunication();

    try {
        while (!(*break_loops))
        {
            // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
            // robot will effectively be in charge of setting the frequency of this loop.
            // In a real-world application this thread should be scheduled with real-time priority in order
            // to ensure that this is called in time.
            std::unique_ptr<rtde_interface::DataPackage> data_pkg(ur_driver->getDataPackage());
            if (data_pkg)
            {
                vector6d_t joint_positions;
                // Read current joint positions from robot data
                if (!data_pkg->getData("actual_q", joint_positions))
                {
                    // This throwing should never happen unless misconfigured
                    std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
                    throw std::runtime_error(error_msg);
                }

                // Store in the thread-safe object
                ur_joint_positions_manager->set_current_joint_positions(joint_positions);


                // Murilo: We should be sure that valid joint positions are available in the buffer.
                if(ur_joint_positions_manager->is_target_joint_position_valid())
                {
                    // Setting the RobotReceiveTimeout time is for example purposes only. This will make the example running more
                    // reliable on non-realtime systems. Use with caution in productive applications.
                    // Murilo: This will run on a realtime system, so I've removed it, i.e., it defaults to 20ms
                    // The target joint positions is obtained from the thread-safe object
                    bool ret = ur_driver->writeJointCommand(ur_joint_positions_manager->get_target_joint_positions(), comm::ControlMode::MODE_SERVOJ);
                    if (!ret)
                    {
                        std::string error_msg = "Could not send joint command. Is the robot in remote control?";
                        throw std::runtime_error(error_msg);
                    }
                }
            }
            else
            {
                URCL_LOG_WARN("Could not get fresh data package from robot");
            }
        }
    } catch (const std::exception& e) {
        //Murilo: Break loops in case of exceptions.
        std::cout << e.what() << std::endl;
        break_loops->store(true);
    }

    ur_driver->stopControl();
    return 0;
}
