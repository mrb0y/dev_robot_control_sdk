/****************************************************************************** 
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
******************************************************************************/
#pragma once

#include "robot_control/fsm/fsm_state.h"
#include <iostream>

/**
 * Passive state
 */
class FSMStatePassive : public FSMState
{
public:
    explicit FSMStatePassive(RobotData *robot_data) : FSMState(robot_data) 
    {
        state_name_ = FSMStateName::PASSIVE;
    }
    ~FSMStatePassive() {}

    // Enter this state
    void on_enter()
    {
        std::cout << "[FSMStatePassive] enter" <<std::endl;
    }

    // Update robot command in the current state
    void run()
    {
        for (int i = 0; i < num_joints_; ++i)
        {
            robot_data_->robot_command.joints[i].kp = 0;
            robot_data_->robot_command.joints[i].kd = 0;
            robot_data_->robot_command.joints[i].tau = 0;
            robot_data_->robot_command.joints[i].vel = 0;
        }
    }

    // Check transition to desired state from user command
    FSMStateName check_transition()
    {
        FSMStateName next_name = state_name_;

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
            // case 3:
            //     next_name = FSMStateName::RL_MODEL;
            //     break;
            default:
                break;
        }

        return next_name;
    }

    // Clean up
    void on_exit()
    {
        std::cout << "[FSMStatePassive] exit" <<std::endl;
    }
};
