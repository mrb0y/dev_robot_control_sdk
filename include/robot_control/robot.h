/******************************************************************************
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
 ******************************************************************************/
#pragma once

#include <stdint.h>
#include <vector>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include "robot_control/robot_data.h"
#include "sensorimotor_interface.h"

/**
 * Hardware abstraction for robot
 */
class Robot
{
public:
    /**
     * Constructor method of class Robot.
     * @param config_path   YAML configure file path.
     */
    Robot(const std::string &config_path);
    virtual ~Robot();

    /**
     * Initialize the robot including its various units.
     * @return 0 for success, others for failure.
     */
    virtual int initialize();

    /**
     * Deinitialize the robot including its various units.
     * @return 0 for success, others for failure.
     */
    virtual void deinitialize();

    /**
     * Enable the robot so we can control it to move.
     * @return 0 for success, others for failure.
     */
    virtual int enable();

    /**
     * Disable the robot to make it enter a low-power standby state immediately.
     * @return 0 for success, others for failure.
     */
    virtual int disable();

    /**
     * Update the current robot state including base pose, joints' position and so on.
     * @return 0 for success, others for failure.
     */
    virtual int update_state();

    /**
     * Send control command to joints.
     * @return 0 for success, others for failure.
     */
    virtual int send_command(const RobotCommand &command);

    /**
     * Get the current robot state.
     * @return the current robot state.
     */
    const RobotState *get_state() { return state_; }

protected:
    /**
     * Callback of ROS subscriber to battery state topic.
     */
    void on_battery_state_callback(const sensor_msgs::BatteryState::ConstPtr &msg);

private:
    Robot() {}

protected:
    RobotState *state_ = nullptr;

    // Config
    YAML::Node config_;
    int num_joints_ = 0;
    std::vector<int> actuator_directions_;
    std::vector<float> joint_zero_pos_;

    // Raw data
    imu_data_t imu_data_;
    std::vector<actuator_data_t> actuator_data_;
    std::vector<actuator_control_parameters_t> actuator_cmd_;

    // ROS
    ros::NodeHandle node_;
    std::vector<ros::Subscriber> subscribers_;

    // Battery state
    sensor_msgs::BatteryState battery_state_;
    std::mutex battery_state_mutex_;
};
