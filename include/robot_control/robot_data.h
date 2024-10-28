/****************************************************************************** 
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
******************************************************************************/
#pragma once

#include <stdint.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/BatteryState.h>
#include "yaml-cpp/yaml.h"

/**
 * Joint state data
 */
struct JointState
{
    // position. uinit: rad
	float pos = 0.0f;

    // velocity. uinit: rad/s
	float vel = 0.0f;

    // torque. uinit: Nm
	float tau = 0.0f;

    // Online state. 1 is online, 0 is offline
    uint8_t online = 0;

    // Running mode. 1 is enabled, 0 is disabled
    uint8_t mode = 0;

    // ActuatorAlarmCode
    uint8_t alarm = 0;

    // temperature. uinit: degrees
    uint8_t temperature = 0;
};

/**
 * Robot base state data
 */
struct BaseState
{
    // Acceleration. uinit: m/s2
	float acc_x = 0.0f;
	float acc_y = 0.0f;
	float acc_z = 0.0f;

    // Angular velocity. uinit: rad/s
	float gyro_x = 0.0f;
	float gyro_y = 0.0f;
	float gyro_z = 0.0f;

    // Orientation. uinit: rad
	float pitch = 0.0f;
	float roll = 0.0f;
	float yaw = 0.0f;
	
    // Orientation in quaternion
	float quaternion_w = 0.0f;
	float quaternion_x = 0.0f;
	float quaternion_y = 0.0f;
	float quaternion_z = 0.0f;

    // Position, velocity and more
};

/**
 * Robot state data
 */
struct RobotState
{
    BaseState base;
    std::vector<JointState> joints;
    sensor_msgs::BatteryState battery;
};

/**
 * Joint command data
 */
struct JointCommand
{
    // Desired position. uinit: rad
	float pos = 0.0f;

    // Desired velocity. uinit: rad/s
	float vel = 0.0f;

    // Feedforward torque. uinit: Nm
	float tau = 0.0f;

    // Position gain. uinit: Nm/rad
    float kp = 0.0f;

    // Velocity gain. uinit: Nm/(rad/s)
    float kd = 0.0f;
};

/**
 * Robot command data
 */
struct RobotCommand
{
    std::vector<JointCommand> joints;
};


/**
 * Remote command data. You can add more fields in this struct.
 */
struct RemoteCommand
{
    // Mode
    int mode[3] = {0};

    // Velocity command
    float vx = 0.0;
    float vy = 0.0;
    float wz = 0.0;
};


/**
 * Robot data
 */
struct RobotData
{
    RobotState robot_state;
    RobotCommand robot_command;
    RemoteCommand remote_command;
    YAML::Node config;
};

#define DEGREE_TO_RAD(v) (v * M_PI / 180)
