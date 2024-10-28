/****************************************************************************** 
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
******************************************************************************/
#pragma once

#include "robot_control/robot.h"
#include "robot_control/remote_control.h"
#include "fsm/fsm_state.h"


class RobotControl
{
public:
    RobotControl(const std::string &config_path);
    ~RobotControl();

    /**
     * Initialize robot control
     * @return 0 for success, others for failure
     */
    int initialize();

    /**
     * Deinitialize robot control
     */
    void deinitialize();

    /**
     * Start robot control (thread) loop
     * @return 0 for success, others for failure
     */
    int start();

    /**
     * Stop robot control (thread) loop
     */
    void stop();

protected:
    /**
     * Thread callback 
     */
    void run();

    /**
     * Update state data and user's high level command 
     * @return 0 for success, others for failure
     */
    int update_data();

    /**
     * Run Finite State Machine
     */
    void run_fsm();

    /**
     * Send low level command to robot joints 
     * @return 0 for success, others for failure
     */
    int send_command();

    /**
     * Print data 
     */
    void print_data();

    /**
     * Record data 
     */
    void record_data();

    /**
     * Start recording data 
     */
    void start_data_record();

    /**
     * Stop recording data 
     */
    void stop_data_record();

private:
    /**
     * Not used
     */
    RobotControl() {}

protected:
    // Data
    RobotData data_;
    FILE *record_ = nullptr;

    // Devices
    Robot *robot_ = nullptr;
    RemoteControl *remote_controller_ = nullptr;

    // Running thread
    float dt_ = 0.005f;
    bool running_ = false;
    std::thread control_thread_;

    // Finite State Machine
    FSMStateList fsm_;
    FSMState *current_state_ = nullptr;
    bool first_run_ = true;
};
