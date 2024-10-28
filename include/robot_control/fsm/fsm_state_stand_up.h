/****************************************************************************** 
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
******************************************************************************/
#pragma once

#include "robot_control/fsm/fsm_state.h"
#include <iostream>

/**
 * StandUp state
 */
class FSMStateStandUp : public FSMState
{
public:
    explicit FSMStateStandUp(RobotData *robot_data) : FSMState(robot_data) 
    {
        state_name_ = FSMStateName::STAND_UP;

        // Initialize stand up pose
        const YAML::Node &stand_up = robot_data_->config["stand_up"];
        for (int i = 0; i < num_joints_; ++i) 
        {
            target_pose_.push_back(stand_up["pose"][i].as<float>());
            kp_.push_back(stand_up["kp"][i].as<float>());
            kd_.push_back(stand_up["kd"][i].as<float>());
        }
        duration_ = stand_up["duration"].as<float>();
    }
    ~FSMStateStandUp() {}

    // Enter this state
    void on_enter()
    {
        std::cout << "[FSMStateStandUp] enter" <<std::endl;

        // Set initial pose
        initial_pose_.resize(num_joints_);

        for (int i = 0; i < num_joints_; ++i)
        {
            initial_pose_[i] = robot_data_->robot_state.joints[i].pos;
        }

        start_time_ = get_monotonic_time();
    }

    // Update robot command in the current state
    void run()
    {
        float progress = std::min(1.0, std::max(0.0, (get_monotonic_time() - start_time_) / duration_));

        for (int i = 0; i < num_joints_; ++i)
        {
            robot_data_->robot_command.joints[i].kp = kp_[i];
            robot_data_->robot_command.joints[i].kd = kd_[i];
            robot_data_->robot_command.joints[i].pos = (1 - progress) * initial_pose_[i] + progress * target_pose_[i];
            robot_data_->robot_command.joints[i].vel = 0;
            robot_data_->robot_command.joints[i].tau = 0;
        }
    }

    // Check transition to desired state from user command
    FSMStateName check_transition()
    {
        FSMStateName next_name = state_name_;

        if (!check_safety())
        {
            return FSMStateName::SOFT_STOP;
        }

        switch (robot_data_->remote_command.mode[0])
        {
            case 0:
                next_name = FSMStateName::PASSIVE;
                break;
            case 1:
                next_name = FSMStateName::LIE_DOWN;
                break;
            case 2:
                next_name = FSMStateName::STAND_UP;
                break;
            case 3:
                next_name = FSMStateName::RL_MODEL;
                break;
            case 4:
                next_name = FSMStateName::SOFT_STOP;
                break;
            default:
                break;
        }

        return next_name;
    }
    
    // Clean up
    void on_exit()
    {
        std::cout << "[FSMStateStandUp] exit" <<std::endl;
    }

protected:
    std::vector<float> target_pose_;
    std::vector<float> kp_;
    std::vector<float> kd_;
    double duration_ = 6.0;
    double start_time_ = 0.0;
    std::vector<float> initial_pose_;
};
