# Dev Robot Control SDK Documentation

This document provides an overview of the Dev Robot Control SDK, its architecture, configuration, and usage.

## Overview

The Dev Robot Control SDK is a C++ application built using ROS Noetic for controlling WEILAN Dev series robots (specifically the "DevQ" model mentioned in the configuration). It utilizes a Finite State Machine (FSM) to manage different motion modes and interfaces with the robot's hardware (actuators, IMU) via the `sensorimotor_interface` library.

## Architecture

The SDK follows a layered architecture:

1.  **Entry Point (`main.cpp`):**
    *   Initializes the ROS node (`robot_control`).
    *   Parses the path to the YAML configuration file from command-line arguments.
    *   Creates an instance of the `RobotControl` class.
    *   Initializes, starts, and manages the lifecycle of the `RobotControl` object.
    *   Enters the main ROS spin loop (`ros::spin()`).

2.  **Main Controller (`RobotControl`):**
    *   Located in `include/robot_control/robot_control.h` and `src/robot_control.cpp`.
    *   Orchestrates the main control logic within a dedicated thread running at a fixed frequency (defined by `dt` in the config).
    *   **Finite State Machine (FSM):** Manages robot behavior through different states (defined in `include/robot_control/fsm/`):
        *   `PASSIVE` (0): Robot is idle.
        *   `LIE_DOWN` (1): Robot performs a lie-down maneuver.
        *   `STAND_UP` (2): Robot performs a stand-up maneuver.
        *   `RL_MODEL` (3): Robot control is potentially driven by a Reinforcement Learning model (details TBD, uses `model/devq/policy.mnn`).
        *   `SOFT_STOP` (4): Robot comes to a gentle stop.
    *   **Remote Control:** Reads high-level commands (target velocity, mode changes) from a `RemoteControl` object. The default implementation (`RosRemoteControl`) subscribes to ROS topics.
    *   **Robot Interface:** Interacts with the `Robot` object to get current state (IMU, joints, battery) and send low-level joint commands.
    *   **Control Loop (`run()`):**
        1.  Waits for the timer tick (`dt`).
        2.  Updates robot state (`update_data()` calls `robot_->update_state()`).
        3.  Gets remote commands (`update_data()` calls `remote_controller_->get_command()`).
        4.  Runs the FSM (`run_fsm()`), which checks for state transitions and executes the current state's logic, ultimately calculating desired joint commands (`data_.robot_command`).
        5.  Sends commands to the robot hardware (`send_command()` calls `robot_->send_command()`).
        6.  Optionally prints state data (`print_data()`) and records data to a CSV file (`record_data()`).

3.  **Hardware Abstraction (`Robot`):**
    *   Located in `include/robot_control/robot.h` and `src/robot.cpp`.
    *   Provides an abstraction layer over the physical hardware.
    *   Uses the `sensorimotor_interface` library (pre-compiled `.so` files in `sensorimotor_interface/lib/`) to communicate with actuators (via SPI) and the IMU.
    *   Reads configuration (joint count, directions, zero offsets, device paths) from the YAML file.
    *   Subscribes to ROS topics for external state information (e.g., battery state via `sensor_msgs/BatteryState`).
    *   `update_state()`: Reads data from actuators and IMU using `sensorimotor_interface` functions (`get_actuators_data`, `get_imu_data`) and updates the internal `RobotState` structure.
    *   `send_command()`: Takes a `RobotCommand` structure and sends it to the actuators using `sensorimotor_interface` functions (`send_command_to_actuators_async`), applying necessary conversions (directions, zero offsets).

4.  **Remote Control Interface (`RemoteControl`):**
    *   Located in `include/robot_control/remote_control.h` and `src/remote_control.cpp`.
    *   Abstract base class defining the interface for receiving external commands.
    *   `RosRemoteControl`: Default implementation subscribing to ROS topics to receive commands.

## Data Flow

*   **Command Input:**
    *   ROS Topics (`/robot_control/set_mode`, `/robot_control/set_velocity`) -> `RosRemoteControl` -> `RobotControl::data_.remote_command`
*   **Control Logic:**
    *   `RobotControl::data_.remote_command` & `RobotControl::data_.robot_state` -> FSM States -> `RobotControl::data_.robot_command`
*   **Command Output:**
    *   `RobotControl::data_.robot_command` -> `Robot::send_command()` -> `sensorimotor_interface` -> Actuators
*   **State Feedback:**
    *   Actuators & IMU -> `sensorimotor_interface` -> `Robot::update_state()` -> `RobotControl::data_.robot_state`
    *   Battery Sensor -> ROS Topic -> `Robot::on_battery_state_callback()` -> `RobotControl::data_.robot_state`

## Configuration (`config/devq.yaml`)

This YAML file contains crucial parameters:

*   `robot_type`: Identifier for the robot model.
*   `num_joints`: Number of controllable joints.
*   `dt`: Control loop period in seconds (e.g., 0.005 for 200Hz).
*   `spi_dev`: Path to the SPI device for actuator communication (e.g., `/dev/spidev1.0`).
*   `imu_dev`: Path to the IMU device.
*   `set_homing`: Boolean flag for actuator initialization.
*   `actuator_directions`: Array specifying the direction multiplier (+1 or -1) for each joint.
*   `joint_zero_pos`: Array specifying the zero offset (in degrees) for each joint.
*   `battery_state_topic`: ROS topic name for battery state messages (e.g., `/alphadog_aux/battery_state`).
*   `remote_control_type`: Specifies the input method. Set to `"ros"` (default) to use ROS topics or `"bluetooth"` to use a Bluetooth gamepad.
*   `bluetooth_device_address`: Required if `remote_control_type` is `"bluetooth"`. Set to the MAC address of the paired Bluetooth controller (e.g., `"XX:XX:XX:XX:XX:XX"`).

## Bluetooth Controller Setup (Optional)

To use a Bluetooth controller instead of ROS topics for input:

1.  **Install BlueZ Development Library:**
    ```bash
    sudo apt update
    sudo apt install libbluetooth-dev
    ```
2.  **Find Controller MAC Address:**
    *   Turn on your Bluetooth controller and put it in pairing mode.
    *   Use the `bluetoothctl` command-line tool:
        ```bash
        bluetoothctl
        # Inside bluetoothctl:
        scan on 
        # Wait for your controller to appear, note its MAC address (e.g., AA:BB:CC:11:22:33)
        scan off
        exit
        ```
3.  **Pair and Trust the Controller:**
    *   Still using `bluetoothctl`:
        ```bash
        bluetoothctl
        # Inside bluetoothctl:
        pair AA:BB:CC:11:22:33   # Replace with your controller's address
        trust AA:BB:CC:11:22:33  # Trust the device for auto-reconnection
        connect AA:BB:CC:11:22:33 # Connect to test (optional, SDK will connect)
        exit
        ```
    *   Ensure the controller is paired and trusted by the Ubuntu system. You might need to use the system's Bluetooth GUI settings as well.
4.  **Configure `devq.yaml`:**
    *   Set `remote_control_type: "bluetooth"`.
    *   Set `bluetooth_device_address:` to the MAC address you found (e.g., `"AA:BB:CC:11:22:33"`).
5.  **Rebuild the SDK:** If you haven't built since adding the Bluetooth code:
    ```bash
    cd ~/example_ws 
    catkin_make install
    ```
6.  **Run the SDK:** Follow the standard run instructions. The SDK will attempt to connect to the specified Bluetooth controller.

### Bluetooth Control Mapping

*   **Left Stick Vertical:** Forward/Backward Velocity (`vx`)
*   **Left Stick Horizontal:** Sideways Velocity (`vy`)
*   **Right Stick Horizontal:** Turning Velocity (`wz`)
*   **Start Button:** Press to cycle through FSM modes (Passive -> Lie Down -> Stand Up -> RL Model -> Soft Stop -> Passive...)

*Note: The exact axis and button indices used in the code (`AXIS_LEFT_Y`, `BUTTON_START`, etc.) might need adjustment depending on your specific controller model. The input processing in `src/bluetooth_remote_control.cpp` is currently a placeholder and may require significant refinement based on the controller's data protocol.*

## Build Instructions

(Assumes Ubuntu 20.04, ROS Noetic, Eigen3, MNN are installed)

```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash

# Create and navigate to Catkin workspace
mkdir -p ~/example_ws/src
cd ~/example_ws

# Clone the repository (if not already present)
# git clone https://github.com/AlphaDogDeveloper/dev_robot_control_sdk.git src/robot_control

# Build the workspace
catkin_make install
```

## Run Instructions

**Important:** Ensure no other program is controlling the robot hardware.

```bash
# Navigate to the workspace
cd ~/example_ws

# Source the installed environment
source install/setup.bash 

# Navigate to the library directory
cd install/lib/robot_control

# Set library path (needed for sensorimotor_interface.so)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:./

# Run the control program
./robot_control config/devq.yaml 
```

## ROS API

### Subscribed Topics

*   `/robot_control/set_mode` (`std_msgs/Int32MultiArray`): Sets the desired FSM mode. Send `data: [mode_id]`.
    *   `0`: PASSIVE
    *   `1`: LIE_DOWN
    *   `2`: STAND_UP
    *   `3`: RL_MODEL
    *   `4`: SOFT_STOP
*   `/robot_control/set_velocity` (`std_msgs/Float32MultiArray`): Sets the desired robot velocity. Send `data: [vx, vy, wz]`.
*   *(Configurable)* Battery state topic (e.g., `/battery_state`, type `sensor_msgs/BatteryState`): Used internally by the `Robot` class.

### Published Topics

*   `/robot_control/telemetry/imu` (`sensor_msgs/Imu`): Publishes real-time IMU data including orientation (quaternion), angular velocity, and linear acceleration. Frame ID is set to `imu_link`. Published at the main control loop frequency.
*   `/robot_control/telemetry/joints` (`sensor_msgs/JointState`): Publishes real-time joint states including position, velocity, and effort (torque). Joint names are assumed to follow the pattern `joint_0`, `joint_1`, etc. Published at the main control loop frequency.
*   `/robot_control/telemetry/battery` (`sensor_msgs/BatteryState`): Re-publishes the latest received battery state information (voltage, current, percentage, etc.). Published at the main control loop frequency.
*   `/robot_control/telemetry/fsm_state` (`std_msgs/String`): Publishes the name of the currently active FSM state (e.g., "PASSIVE", "STAND_UP"). Published at the main control loop frequency.
