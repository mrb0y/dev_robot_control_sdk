/******************************************************************************
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
 ******************************************************************************/
#pragma once

#include "robot_control/remote_control.h"
#include <string>
#include <thread>
#include <atomic>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

/**
 * Remote controller based on Bluetooth communication with a gamepad.
 */
class BluetoothRemoteControl : public RemoteControl
{
public:
    /**
     * Constructor.
     * @param device_address MAC address of the Bluetooth gamepad.
     */
    explicit BluetoothRemoteControl(const std::string &device_address);
    virtual ~BluetoothRemoteControl();

    /**
     * Start the Bluetooth communication thread.
     * @return 0 on success, negative on error.
     */
    int start();

    /**
     * Stop the Bluetooth communication thread.
     */
    void stop();

private:
    /**
     * Main loop for the Bluetooth communication thread.
     */
    void communication_thread_func();

    /**
     * Process raw input event data from the controller.
     * This needs to be implemented based on the specific controller's protocol.
     * For simplicity, we'll assume a basic structure for now.
     * @param buffer Raw data buffer.
     * @param len Length of data in buffer.
     */
    void process_input(const char* buffer, int len);

    /**
     * Cycles the command mode to the next state.
     */
    void cycle_mode();

private:
    std::string device_address_;
    std::thread communication_thread_;
    std::atomic<bool> running_;
    int socket_ = -1;
    int current_mode_index_ = 0; // To track cycling through modes

    // Define constants for button/axis mapping (example for a generic controller)
    // These would need adjustment based on the actual controller event data
    static constexpr int AXIS_LEFT_Y = 1;
    static constexpr int AXIS_LEFT_X = 0;
    static constexpr int AXIS_RIGHT_X = 3; // Or 2, depending on controller
    static constexpr int BUTTON_START = 9; // Example button index

    static constexpr float AXIS_MAX = 32767.0f; // Max value for joystick axis
};
