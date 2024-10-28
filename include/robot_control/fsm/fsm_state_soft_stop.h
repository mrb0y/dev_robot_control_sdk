/******************************************************************************
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
 ******************************************************************************/
#pragma once

#include "robot_control/fsm/fsm_state.h"
#include <iostream>

/**
 * SoftStop state
 */
class FSMStateSoftStop : public FSMState
{
public:
    explicit FSMStateSoftStop(RobotData *robot_data) : FSMState(robot_data)
    {
        state_name_ = FSMStateName::SOFT_STOP;

        // Initialize lie down pose
        const YAML::Node &soft_stop = robot_data_->config["soft_stop"];
        for (int i = 0; i < num_joints_; ++i)
        {
            kd_.push_back(soft_stop["kd"][i].as<float>());
        }
        duration_ = soft_stop["duration"].as<float>();
    }
    ~FSMStateSoftStop() {}

    // Enter this state
    void on_enter()
    {
        std::cout << "[FSMStateSoftStop] enter" << std::endl;

        start_time_ = get_monotonic_time();
    }

    // Update robot command in the current state
    void run()
    {
        float soft_scale = 1.0 - std::min(1.0, std::max(0.0, (get_monotonic_time() - start_time_) / duration_ * 2 - 1));

        for (int i = 0; i < num_joints_; ++i)
        {
            robot_data_->robot_command.joints[i].kp = 0;
            robot_data_->robot_command.joints[i].kd = kd_[i] * soft_scale;
            robot_data_->robot_command.joints[i].vel = 0;
            robot_data_->robot_command.joints[i].tau = 0;
        }
    }

    // Check transition to desired state from user command
    FSMStateName check_transition()
    {
        FSMStateName next_name = state_name_;

        if (!check_safety(true))
        {
            return FSMStateName::SOFT_STOP;
        }

        switch (robot_data_->remote_command.mode[0])
        {
            // Allow transition to PASSIVE only in SOFT_STOP mode
            case 0:
                next_name = FSMStateName::PASSIVE;
                break;
            default:
                break;
        }

        return next_name;
    }

    // Clean up
    void on_exit()
    {
        std::cout << "[FSMStateSoftStop] exit" << std::endl;
    }

protected:
    std::vector<float> kd_;
    double duration_ = 6.0;
    double start_time_ = 0.0;
};
