/****************************************************************************** 
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
******************************************************************************/
#pragma once
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "robot_control/robot_data.h"

/**
 * Abstract class for any kind of remote controller.
 */
class RemoteControl
{
public:
    RemoteControl();
    virtual ~RemoteControl();

    void get_command(RemoteCommand &cmd);

protected:
    RemoteCommand cmd_;
    std::mutex cmd_mutex_;
};

/**
 * Remote controller base on ROS communication.
 */
class RosRemoteControl: public RemoteControl
{
public:
    RosRemoteControl();
    virtual ~RosRemoteControl();

protected:
    void on_set_mode_callback(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void on_set_velocity_callback(const std_msgs::Float32MultiArray::ConstPtr &msg);

protected:
    ros::NodeHandle node_;
    std::vector<ros::Subscriber> subscribers_;
};