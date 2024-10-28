# Dev Robot Control SDK

An example of motion control program for WEILAN Dev series robots.

SDK directory structure:

- [sensorimotor_interface](https://github.com/AlphaDogDeveloper/dev_robot_control_sdk/blob/main/sensorimotor_interface) C language interface and library for communication with sensorimotors.
- [config](https://github.com/AlphaDogDeveloper/dev_robot_control_sdk/blob/main/config) Robot control configure files.
- [include](https://github.com/AlphaDogDeveloper/dev_robot_control_sdk/blob/main/include) C++ header files of the example motion control program.
- [src](https://github.com/AlphaDogDeveloper/dev_robot_control_sdk/blob/main/src) C++ source code of the example motion control program.
- [third-party](https://github.com/AlphaDogDeveloper/dev_robot_control_sdk/blob/main/third-party) Third-party libraries and other dependences.
- [model](https://github.com/AlphaDogDeveloper/dev_robot_control_sdk/blob/main/model) Learning based model files.
- [resources](https://github.com/AlphaDogDeveloper/dev_robot_control_sdk/blob/main/resources) Assets including URDF and meshes of robots.

## Build

Prerequisites (All the following have already been ready on WEILAN DevQ robot mainboard)

- Ubuntu 20.04
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Eigen3](http://eigen.tuxfamily.org/index.php)
- [MNN](https://github.com/alibaba/MNN)

Compile the SDK

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/example_ws/src
cd ~/example_ws
git clone git@github.com:AlphaDogDeveloper/dev_robot_control_sdk.git src/robot_control
catkin_make install
```

## Run 

### Run robot control program

```bash
cd ~/example_ws
source install/setup.bash 
cd install/lib/robot_control
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:./
./robot_control config/devq.yaml
```

### Test setting motion mode

| mode name | mode ID |
| --- | --- |
| PASSIVE | 0 |
| LIE_DOWN | 1 |
| STAND_UP | 2 |
| RL_MODEL | 3 |
| SOFT_STOP | 4 |

```bash
source /opt/ros/noetic/setup.bash
# Set motion mode to STAND_UP
rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [2]"
```

```bash
source /opt/ros/noetic/setup.bash
# Set motion mode to LIE_DOWN
rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [1]"
```

### Test setting velocity

```bash
source /opt/ros/noetic/setup.bash
# Set vx=0.1, vy=0.0, wz=0.3
rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.1, 0.0, 0.3]"
```
