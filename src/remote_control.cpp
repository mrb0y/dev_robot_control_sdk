/****************************************************************************** 
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
******************************************************************************/
#include "robot_control/remote_control.h"
#include <memory.h>

RemoteControl::RemoteControl()
{
}

RemoteControl::~RemoteControl()
{
}

void RemoteControl::get_command(RemoteCommand &cmd)
{
    cmd_mutex_.lock();
    memcpy(&cmd, &cmd_, sizeof(RemoteCommand));
    cmd_mutex_.unlock();
}

RosRemoteControl::RosRemoteControl() : node_("robot_control")
{
    subscribers_.emplace_back(node_.subscribe("set_mode", 5, &RosRemoteControl::on_set_mode_callback, this));
    subscribers_.emplace_back(node_.subscribe("set_velocity", 5, &RosRemoteControl::on_set_velocity_callback, this));
}

RosRemoteControl::~RosRemoteControl()
{
}

void RosRemoteControl::on_set_mode_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    std::cout << "[RosRemoteControl] set mode ";
    for (auto it = msg->data.begin(); it != msg->data.end(); it++)
    {
        std::cout << *it << ",";
    }
    std::cout << std::endl;

    cmd_mutex_.lock();
    for (int i = 0; i < std::min(3, static_cast<int>(msg->data.size())); ++i)
    {
        cmd_.mode[i] = msg->data[i];
    }
    cmd_mutex_.unlock();
}

void RosRemoteControl::on_set_velocity_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    std::cout << "[RosRemoteControl] set velocity ";
    for (auto it = msg->data.begin(); it != msg->data.end(); it++)
    {
        std::cout << *it << ",";
    }
    std::cout << std::endl;

    int size = static_cast<int>(msg->data.size());
    cmd_mutex_.lock();
    if (size >= 1)
    {
        cmd_.vx = msg->data[0];

        if (size >= 2)
        {
            cmd_.vy = msg->data[1];

            if (size >= 3)
            {
                cmd_.wz = msg->data[2];
            }
        }
    }
    cmd_mutex_.unlock();
}
