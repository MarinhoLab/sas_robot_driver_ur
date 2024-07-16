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

#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include <iostream>
#include <memory>

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

//vector6d_t g_joint_positions;

// We need a callback function to register. See UrDriver's parameters for details.
void handleRobotProgramState(bool program_running)
{
    // Print the text in green so we see it better
    std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

int main(int argc, char* argv[])
{
    urcl::setLogLevel(urcl::LogLevel::INFO);

    // Parse the ip arguments if given
    std::string robot_ip = DEFAULT_ROBOT_IP;
    if (argc > 1)
    {
        robot_ip = std::string(argv[1]);
    }

    // Making the robot ready for the program by:
    // Connect the the robot Dashboard
    g_my_dashboard.reset(new DashboardClient(robot_ip));
    if (!g_my_dashboard->connect())
    {
        URCL_LOG_ERROR("Could not connect to dashboard");
        return 1;
    }

    // Stop program, if there is one running
    if (!g_my_dashboard->commandStop())
    {
        URCL_LOG_ERROR("Could not send stop program command");
        return 1;
    }

    // Power it off
    if (!g_my_dashboard->commandPowerOff())
    {
        URCL_LOG_ERROR("Could not send Power off command");
        return 1;
    }

    // Power it on
    if (!g_my_dashboard->commandPowerOn())
    {
        URCL_LOG_ERROR("Could not send Power on command");
        return 1;
    }

    // Release the brakes
    if (!g_my_dashboard->commandBrakeRelease())
    {
        URCL_LOG_ERROR("Could not send BrakeRelease command");
        return 1;
    }

    // Now the robot is ready to receive a program

    std::unique_ptr<ToolCommSetup> tool_comm_setup;
    const bool HEADLESS = true;
    g_my_driver.reset(new UrDriver(robot_ip, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState, HEADLESS,
                                   std::move(tool_comm_setup), CALIBRATION_CHECKSUM));

}


int communication_thread_loop(std::shared_ptr<UrDriver> ur_driver,
                              std::atomic_bool* break_loops)
{
    //Murilo: despite the many modifications, I've left the original comments from the example, hoping that they might be useful in the future.
    vector6d_t joint_positions;

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
            std::unique_ptr<rtde_interface::DataPackage> data_pkg = ur_driver->getDataPackage();
            if (data_pkg)
            {
                // Read current joint positions from robot data
                if (!data_pkg->getData("actual_q", joint_positions))
                {
                    // This throwing should never happen unless misconfigured
                    std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
                    throw std::runtime_error(error_msg);
                }

                // Setting the RobotReceiveTimeout time is for example purposes only. This will make the example running more
                // reliable on non-realtime systems. Use with caution in productive applications.
                // Murilo: This will run on a realtime system, so I've removed it, i.e., it defaults to 20ms
                bool ret = ur_driver->writeJointCommand(joint_positions, comm::ControlMode::MODE_SERVOJ);
                if (!ret)
                {
                    std::string error_msg = "Could not send joint command. Is the robot in remote control?";
                    throw std::runtime_error(error_msg);
                }
            }
            else
            {
                URCL_LOG_WARN("Could not get fresh data package from robot");
            }
        }
    } catch (...) {
        break_loops->store(false);
    }

    ur_driver->stopControl();
    return 0;
}
