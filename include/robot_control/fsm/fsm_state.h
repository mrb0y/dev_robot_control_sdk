/******************************************************************************
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
 ******************************************************************************/
#pragma once

#include "robot_control/robot_data.h"

/**
 * FSM state name
 */
enum FSMStateName
{
    PASSIVE = 0,
    LIE_DOWN,
    STAND_UP,
    RL_MODEL,
    SOFT_STOP
};

/**
 * FSM state
 */
class FSMState
{
public:
    explicit FSMState(RobotData *robot_data) : robot_data_(robot_data)
    {
        num_joints_ = robot_data->robot_state.joints.size();
    }

    virtual ~FSMState() {}

    // Enter this state
    virtual void on_enter() = 0;

    // Update robot command in the current state
    virtual void run() = 0;

    // Check transition to desired state from user command
    virtual FSMStateName check_transition() = 0;

    // Clean up
    virtual void on_exit() = 0;

protected:
    // Check whether need stop for safety
    virtual bool check_safety(bool silence = false)
    {
        for (size_t i = 0; i < num_joints_; ++i)
        {
            // Any actuator is offline
            if (0 == robot_data_->robot_state.joints[i].online)
            {
                if (!silence)
                {
                    std::cout << "[FSMState] Joint " << i << " is not online!" << std::endl;
                }
                return false;
            }

            // Any actuator has alarm other than ActuatorAlarmCode::AAC_DESIRED_TORQUE_OUT_OF_LIMIT
            if (0 != (robot_data_->robot_state.joints[i].alarm & ~ActuatorAlarmCode::AAC_DESIRED_TORQUE_OUT_OF_LIMIT))
            {
                if (!silence)
                {
                    std::cout << "[FSMState] Joint " << i << " alarm " << robot_data_->robot_state.joints[i].alarm << "!" << std::endl;
                }
                return false;
            }

            // Battery percentage is very low and battery is still discharging
            if (robot_data_->robot_state.battery.percentage < 0.1f && robot_data_->robot_state.battery.current < 0.f)
            {
                if (!silence)
                {
                    std::cout << "[FSMState] Battery percentage " << robot_data_->robot_state.battery.percentage << " is very low and current is " << robot_data_->robot_state.battery.current << "!" << std::endl;
                }
                return false;
            }
        }

        return true;
    }

public:
    FSMStateName state_name_;

protected:
    RobotData *robot_data_;
    int num_joints_;
};

/**
 * FSM state list
 */
struct FSMStateList
{
    FSMState *passive = nullptr;
    FSMState *lie_down = nullptr;
    FSMState *stand_up = nullptr;
    FSMState *rl_model = nullptr;
    FSMState *soft_stop = nullptr;

    /**
     * Get state by name
     * @param state_name FSMStateName
     * @return FSM state with name `state_name` or nullptr if not found
     */
    FSMState *get_state_by_name(FSMStateName state_name)
    {
        FSMState *state;
        switch (state_name)
        {
        case FSMStateName::PASSIVE:
            state = passive;
            break;
        case FSMStateName::LIE_DOWN:
            state = lie_down;
            break;
        case FSMStateName::STAND_UP:
            state = stand_up;
            break;
        case FSMStateName::RL_MODEL:
            state = rl_model;
            break;
        case FSMStateName::SOFT_STOP:
            state = soft_stop;
            break;
        default:
            state = nullptr;
            break;
        }
        return state;
    }
};

/**
 * Get monotonic time in seconds
 */
inline double get_monotonic_time()
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec + now.tv_nsec / 1e9;
}
