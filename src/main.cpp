/****************************************************************************** 
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "robot_control/robot_control.h"

int main(int argc, char **argv)
{
    int err;

    ros::init(argc, argv, "robot_control", ros::init_options::NoRosout);

    // Parse args
    std::string config_path;
    if (argc > 1) 
    {
        config_path = argv[1];
    }
    else
    {
        printf("Usage: ./robot_control [YAML config file path]\n");
        return 1;
    }

    // Create robot control
    RobotControl robot_control(config_path);

    // Initialize robot controller
    err = robot_control.initialize();
    if (err != 0)
    {
        printf("[main] Failed to initialize (error:%d).\n", err);
        return 2;
    }

    // Start robot controller
    err = robot_control.start();
    if (err != 0)
    {
        printf("[main] Failed to start (error:%d).\n", err);
        return 3;
    }

    // Spin
    ros::spin();

    // Stop robot controller
    robot_control.stop();

    // Deinitialize robot controller
    robot_control.deinitialize();
    return 0;
}