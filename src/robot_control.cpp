/******************************************************************************
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
 ******************************************************************************/
#include "robot_control/robot_control.h"
#include "robot_control/remote_control.h" // Already included via robot_control.h but good practice
#include "robot_control/bluetooth_remote_control.h" // Include the new controller
#include "robot_control/fsm/fsm_state_passive.h"
#include "robot_control/fsm/fsm_state_lie_down.h"
#include "robot_control/fsm/fsm_state_stand_up.h"
#include "robot_control/fsm/fsm_state_rl_model.h"
#include "robot_control/fsm/fsm_state_soft_stop.h"
#include <math.h>
#include <sys/timerfd.h>
#include <unistd.h> // For close() and read()
#include <sys/time.h> // For gettimeofday()
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>

#define ENABLE_RECORD_DATA 1
#define ENABLE_PRINT_DATA 1
#define PRINT_DATA_PERIOD 2.0 // seconds

RobotControl::RobotControl(const std::string &config_path) : remote_controller_(nullptr) // Initialize pointer
{
    // Load config first
    data_.config = YAML::LoadFile(config_path);

    // Determine remote control type
    std::string rc_type = "ros"; // Default to ROS
    if (data_.config["remote_control_type"]) {
        rc_type = data_.config["remote_control_type"].as<std::string>();
    }

    if (rc_type == "bluetooth") {
        std::string bt_address = "";
        if (data_.config["bluetooth_device_address"]) {
            bt_address = data_.config["bluetooth_device_address"].as<std::string>();
        } else {
            std::cerr << "[RobotControl] Warning: remote_control_type is 'bluetooth' but bluetooth_device_address is missing in config. Attempting connection without address (will likely fail)." << std::endl;
            // Or throw an error? For now, let it try and fail.
        }
        std::cout << "[RobotControl] Using Bluetooth Remote Control (Address: " << (bt_address.empty() ? "None" : bt_address) << ")" << std::endl;
        BluetoothRemoteControl* bt_rc = new BluetoothRemoteControl(bt_address);
        if (bt_rc->start() != 0) {
             std::cerr << "[RobotControl] Failed to start Bluetooth Remote Control thread!" << std::endl;
             // Handle error? Maybe fall back to ROS or throw? For now, continue but it won't work.
             delete bt_rc; // Clean up failed instance
             remote_controller_ = nullptr; // Ensure it's null
        } else {
            remote_controller_ = bt_rc;
        }

    } else { // Default or explicitly "ros"
        std::cout << "[RobotControl] Using ROS Remote Control" << std::endl;
        remote_controller_ = new RosRemoteControl();
    }

    // If remote controller failed to initialize, maybe create a dummy one?
    if (!remote_controller_) {
         std::cerr << "[RobotControl] ERROR: No remote controller initialized! Creating dummy." << std::endl;
         // Create a dummy base class instance that does nothing? Or handle error better.
         // For now, let it potentially crash later if null. A better approach is needed.
         // Let's just create a ROS one as a fallback for now.
         remote_controller_ = new RosRemoteControl();
    }


    robot_ = new Robot(config_path);

    // data_.config = YAML::LoadFile(config_path); // Moved up
    dt_ = data_.config["dt"].as<float>();

    size_t num_joints = robot_->get_state()->joints.size();
    data_.robot_command.joints.resize(num_joints);
    data_.robot_state.joints.resize(num_joints);

    fsm_.passive = new FSMStatePassive(&data_);
    fsm_.lie_down = new FSMStateLieDown(&data_);
    fsm_.stand_up = new FSMStateStandUp(&data_);
    fsm_.rl_model = new FSMStateRLModel(&data_);
    fsm_.soft_stop = new FSMStateSoftStop(&data_);

    current_state_ = fsm_.passive;

    // Initialize Telemetry Publishers
    imu_pub_ = node_handle_.advertise<sensor_msgs::Imu>("/robot_control/telemetry/imu", 10);
    joint_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/robot_control/telemetry/joints", 10);
    battery_pub_ = node_handle_.advertise<sensor_msgs::BatteryState>("/robot_control/telemetry/battery", 10);
    fsm_state_pub_ = node_handle_.advertise<std_msgs::String>("/robot_control/telemetry/fsm_state", 10);
}

RobotControl::~RobotControl()
{
    stop(); // This should handle stopping the Bluetooth thread if needed
    deinitialize();

    delete robot_;
    // remote_controller_ deletion is handled in stop() or here, ensure no double delete
    if (remote_controller_) {
        // If it's Bluetooth, stop might have been called already, but check again
         BluetoothRemoteControl* bt_rc = dynamic_cast<BluetoothRemoteControl*>(remote_controller_);
         if (bt_rc) {
             bt_rc->stop(); // Ensure thread is stopped before delete
         }
        delete remote_controller_;
        remote_controller_ = nullptr;
    }
    delete fsm_.passive;
    delete fsm_.lie_down;
    delete fsm_.stand_up;
    delete fsm_.rl_model;
    delete fsm_.soft_stop;
}

/**
 * Initialize robot control
 * @return 0 for success, others for failure
 */
int RobotControl::initialize()
{
    return robot_->initialize();
}

/**
 * Deinitialize robot control
 */
void RobotControl::deinitialize()
{
    return robot_->deinitialize();
}

/**
 * Start robot control (thread) loop
 * @return 0 for success, others for failure
 */
int RobotControl::start()
{
#if ENABLE_RECORD_DATA
    start_data_record();
#endif

    running_ = true;
    control_thread_ = std::thread(&RobotControl::run, this);
    return 0;
}

/**
 * Stop robot control (thread) loop
 */
void RobotControl::stop()
{
    running_ = false;
    // Stop Bluetooth controller thread if it exists
    BluetoothRemoteControl* bt_rc = dynamic_cast<BluetoothRemoteControl*>(remote_controller_);
    if (bt_rc) {
        bt_rc->stop();
    }

    // Join main control thread
    if (control_thread_.joinable())
    {
        control_thread_.join();
    }

#if ENABLE_RECORD_DATA
    stop_data_record();
#endif
}

/**
 * Thread callback
 */
void RobotControl::run()
{
    int err;
    first_run_ = true;

    // Initialize timer
    uint64_t missed;
    itimerspec ts;
    ts.it_value.tv_sec = (int)dt_;
    ts.it_value.tv_nsec = (int)(std::fmod(dt_, 1.f) * 1e9);
    ts.it_interval.tv_sec = ts.it_value.tv_sec;
    ts.it_interval.tv_nsec = ts.it_value.tv_nsec;

    int timer_fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (0 != timerfd_settime(timer_fd, 0, &ts, nullptr))
    {
        printf("[RobotControl] Failed to set timerfd!\n");
        close(timer_fd);
        return;
    }

    // Enable robot
    robot_->enable();

    // Loop
    while (running_)
    {
        // Update state data and user's high level command
        err = update_data();
        if (err != 0)
        {
            break;
        }

        // Run FSM to update joints command
        run_fsm();

        // Send command to joints
        err = send_command();
        if (err != 0)
        {
            break;
        }

#if ENABLE_PRINT_DATA
        // Print data
        print_data();
#endif

#if ENABLE_RECORD_DATA
        // Record data
        record_data();
#endif

        // Publish Telemetry Data
        ros::Time now = ros::Time::now();

        // IMU
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = "imu_link"; // Or appropriate frame
        imu_msg.orientation.w = data_.robot_state.base.quaternion_w;
        imu_msg.orientation.x = data_.robot_state.base.quaternion_x;
        imu_msg.orientation.y = data_.robot_state.base.quaternion_y;
        imu_msg.orientation.z = data_.robot_state.base.quaternion_z;
        imu_msg.angular_velocity.x = data_.robot_state.base.gyro_x;
        imu_msg.angular_velocity.y = data_.robot_state.base.gyro_y;
        imu_msg.angular_velocity.z = data_.robot_state.base.gyro_z;
        imu_msg.linear_acceleration.x = data_.robot_state.base.acc_x;
        imu_msg.linear_acceleration.y = data_.robot_state.base.acc_y;
        imu_msg.linear_acceleration.z = data_.robot_state.base.acc_z;
        // Covariances are often left as 0 if unknown
        imu_pub_.publish(imu_msg);

        // Joints
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = now;
        size_t num_joints = data_.robot_state.joints.size();
        joint_msg.name.resize(num_joints);
        joint_msg.position.resize(num_joints);
        joint_msg.velocity.resize(num_joints);
        joint_msg.effort.resize(num_joints);
        for (size_t i = 0; i < num_joints; ++i) {
            // Assuming joint names follow a pattern like "joint_0", "joint_1", ...
            // This might need adjustment based on the actual URDF or robot model.
            joint_msg.name[i] = "joint_" + std::to_string(i); 
            joint_msg.position[i] = data_.robot_state.joints[i].pos;
            joint_msg.velocity[i] = data_.robot_state.joints[i].vel;
            joint_msg.effort[i] = data_.robot_state.joints[i].tau;
        }
        joint_pub_.publish(joint_msg);

        // Battery (re-publish the latest received state)
        // Note: This assumes the Robot class updates data_.robot_state.battery correctly
        // If the battery state is only updated in the Robot class callback, 
        // we might need a way to access it directly or have RobotControl subscribe too.
        // For now, assume data_.robot_state.battery is reasonably up-to-date.
        sensor_msgs::BatteryState battery_msg = data_.robot_state.battery; 
        battery_msg.header.stamp = now; // Update timestamp
        battery_pub_.publish(battery_msg);

        // FSM State
        std_msgs::String fsm_state_msg;
        fsm_state_msg.data = current_state_->state_name_string(); // Assuming FSMState has a method to get name as string
        fsm_state_pub_.publish(fsm_state_msg);


        // Wait period
        if (sizeof(missed) != read(timer_fd, &missed, sizeof(missed)))
        {
            printf("[RobotControl] Read timerfd error!\n");
            usleep(dt_ * 1000000);
        }
    }

    // Disable robot
    robot_->disable();

    // Deinitialize timer
    close(timer_fd);
}

/**
 * Update state data and user's high level command
 * @return 0 for success, others for failure
 */
int RobotControl::update_data()
{
    int err;

    // Update robot state
    err = robot_->update_state();
    if (err != 0)
    {
        return err;
    }

    data_.robot_state = *robot_->get_state();

    // Get user remote control command
    remote_controller_->get_command(data_.remote_command);

    return 0;
}

/**
 * Run Finite State Machine
 */
void RobotControl::run_fsm()
{
    // Enter the first state
    if (first_run_)
    {
        current_state_->on_enter();
        first_run_ = false;
        printf("[RobotControl] FSM start!\n");
    }

    // Transform to state as user command
    FSMStateName next_state_name = current_state_->check_transition();

    // States transition
    if (next_state_name != current_state_->state_name_)
    {
        FSMState *next_state = fsm_.get_state_by_name(next_state_name);
        if (next_state)
        {
            current_state_->on_exit();
            current_state_ = next_state;
            current_state_->on_enter();
        }
    }

    // Run
    current_state_->run();
}

/**
 * Send low level command to robot joints
 * @return 0 for success, others for failure
 */
int RobotControl::send_command()
{
    return robot_->send_command(data_.robot_command);
}

/**
 * Print data
 */
void RobotControl::print_data()
{
    static ros::Time last_time;

    ros::Time now = ros::Time::now();

    // Print data every 2 seconds
    if ((now - last_time).toSec() < PRINT_DATA_PERIOD)
    {
        return;
    }

    last_time = now;

    printf("[RobotControl] Battery %.1f%%, %.3fA\n",
           data_.robot_state.battery.percentage * 100, data_.robot_state.battery.current);

    printf("[RobotControl] IMU: roll=%.3f pitch=%.3f yaw=%.3f\n",
           data_.robot_state.base.roll, data_.robot_state.base.pitch, data_.robot_state.base.yaw);

    printf("[RobotControl] Joints pos: ");
    for (size_t i = 0; i < data_.robot_state.joints.size(); ++i)
    {
        printf("%.3f ", data_.robot_state.joints[i].pos);
    }
    printf("\n");

    // Print FSM State too
    printf("[RobotControl] FSM State: %s\n", current_state_->state_name_string()); // Assuming FSMState has a method to get name as string
}

/**
 * Record data
 */
void RobotControl::record_data()
{
    if (record_ == nullptr)
    {
        return;
    }

    char buf[4096];
    auto state = robot_->get_state();

    int len = 0;
    sprintf(&buf[len], "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,",
            state->base.roll, state->base.pitch, state->base.yaw,
            state->base.gyro_x, state->base.gyro_y, state->base.gyro_z,
            state->base.acc_x, state->base.acc_y, state->base.acc_z,
            state->battery.percentage, state->battery.current,
            state->battery.power_supply_status == state->battery.POWER_SUPPLY_STATUS_CHARGING);
    len = strlen(buf);

    for (int i = 0; i < static_cast<int>(robot_->get_state()->joints.size()); i++)
    {
        sprintf(&buf[len], "%f,%f,%f,%u,%f,%f,%f,",
                state->joints[i].pos,
                state->joints[i].vel,
                state->joints[i].tau,
                state->joints[i].temperature,
                data_.robot_command.joints[i].pos,
                data_.robot_command.joints[i].vel,
                data_.robot_command.joints[i].tau);
        len = strlen(buf);
    }
    buf[len - 1] = '\n';
    buf[len] = '\0';

    fwrite(buf, len, 1, record_);
}

void RobotControl::start_data_record()
{
    char buf[4096];
    struct timeval tv;
    gettimeofday(&tv, NULL);
    tm *ltm = localtime(&tv.tv_sec);

    sprintf(buf, "data-%d-%02d-%02d-%02d-%02d-%02d.csv",
            1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);

    printf("[RobotControl] Start recording data to '%s'.\n", buf);
    record_ = fopen(buf, "w");

    int len = 0;
    strcpy(&buf[len], "roll,pitch,yaw,wx,wy,wz,ax,ay,az,soc,current,charging,");
    len = strlen(buf);

    for (int i = 0; i < static_cast<int>(robot_->get_state()->joints.size()); i++)
    {
        sprintf(&buf[len], "pos[%d],vel[%d],tau[%d],temperature[%d],cmd_pos[%d],cmd_vel[%d],cmd_tau[%d],",
                i, i, i, i, i, i, i);
        len = strlen(buf);
    }
    buf[len - 1] = '\n';
    buf[len] = '\0';
    fwrite(buf, len, 1, record_);
}

void RobotControl::stop_data_record()
{
    if (record_)
    {
        fclose(record_);
        record_ = nullptr;
    }
}
