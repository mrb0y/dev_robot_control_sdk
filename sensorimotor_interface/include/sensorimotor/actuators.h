#pragma once
#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1)

/**
 * Robot type
 */
typedef enum
{
    RT_QUADRUPED    = 1,
    RT_BIPED        = 2,
} RobotType;

/**
 * Actuator alarm code
 */
typedef enum
{
    AAC_DRV_ERROR                   = 1,
    AAC_DRV_OVERHEAT                = 2,
    AAC_DRV_OVERCURRENT             = 3,
    AAC_DRV_UNDERVOLTAGE            = 4,
    AAC_POSITION_ERROR              = 5,
    AAC_BATTERY_UNDERVOLTAGE        = 6,
    AAC_CIRCLES_ERROR               = 7,
    AAC_OVERHEAT                    = 8,
    AAC_TEMPERATURE_SENSOR_LOST     = 9,
    AAC_CAN_ALARM                   = 10,
    AAC_DESIRED_TORQUE_OUT_OF_LIMIT = 11,
    AAC_OVERVOLTAGE                 = 12,
    AAC_DRV_CONFIG_ERROR            = 13,
    AAC_UNDERVOLTAGE                = 14,
    AAC_PARAMETERS_ERROR            = 15,
    AAC_CURRENT_SAMPLE_CORRECTION   = 16,
} ActuatorAlarmCode;

/**
 * Actuator data structure.
 */
typedef struct
{
    // Online state. 1 is online, 0 is offline
    uint8_t online;

    // Running mode. 1 is enabled, 0 is disabled
    uint8_t mode;

    // ActuatorAlarmCode
    uint8_t alarm;

    // temperature. uinit: degrees
    uint8_t temperature;

    // Whether following data is valid
    uint8_t valid;

    // position. uinit: rad
	float pos;

    // velocity. uinit: rad/s
	float vel;

    // torque. uinit: Nm
	float tau;

} actuator_data_t;

/**
 * Actuator control parameters.
 * output_torque = kp * (desired_pos - current_pose) + kd * (desired_vel - current_vel) + tau
 */
typedef struct
{
    // Desired position. uinit: rad
	float pos;

    // Desired velocity. uinit: rad/s
	float vel;

    // Feedforward torque. uinit: Nm
	float tau;

    // Position gain. uinit: Nm/rad
    float kp;

    // Velocity gain. uinit: Nm/(rad/s)
    float kd;

} actuator_control_parameters_t;

#pragma pack()

/**
 * Actuators data callback function type. Should never invoke any API here inside this callback.
 * @param data  Data of actuators state, ref to actuator_data_t.
 * @param actuators  Array size of @param `data`. It is 12 for RobotType::RT_QUADRUPED and 6 for RobotType::RT_BIPED
 * @param param Pointer for using in class
 */
typedef void (*actuators_data_callback_t)(const actuator_data_t *data, int actuators, void *param);

/**
 * Initialize actuators.
 * @param robot             Robot type, ref to RobotType.
 * @param device_path       SPI device path.
 * @param calibrate         Calibrate IMU orientation.
 * @param callback          Output actuators state data.
 * @param callback_param    Parameter to callback with.
 * @return 0 for success, others for failure.
 */
int initialize_actuators(RobotType robot, const char *device_path, bool calibrate=false, 
    actuators_data_callback_t callback=NULL, void *callback_param=NULL);

/**
 * Deinitialize actuators.
 */
void deinitialize_actuators();

/**
 * Anable all actuators.
 * @param wait Whether wait all actuators being enabled 
 * @return 0 for success, others for failure.
 */
int enable_actuators(bool wait=true);

/**
 * Disable all actuators.
 * @return 0 for success, others for failure.
 */
int disable_actuators();

/**
 * Send control command to actuators.
 * @param cmd   Control parameters.
 * @param actuators  Array size of @param `cmd`. It should be 12 for RobotType::RT_QUADRUPED and 6 for RobotType::RT_BIPED
 * @return 0 for success, others for failure.
 */
int send_command_to_actuators(const actuator_control_parameters_t *cmd, size_t actuators);

/**
 * Send control command to actuators asynchronously.
 * @param cmd   Control parameters.
 * @param actuators  Array size of @param `cmd`. It should be 12 for RobotType::RT_QUADRUPED and 6 for RobotType::RT_BIPED
 * @return 0 for success, others for failure.
 */
int send_command_to_actuators_async(const actuator_control_parameters_t *cmd, size_t actuators);

/**
 * Get the latest state of actuators.
 * @param data  Return current state data of actuators. It should have enough array size for all actuators of the current robot type
 * @param actuators  Array size of @param `data`. It should be 12 for RobotType::RT_QUADRUPED and 6 for RobotType::RT_BIPED
 * @return 0 for success, others for failure.
 */
int get_actuators_data(actuator_data_t *data, size_t actuators);

#ifdef __cplusplus
}
#endif

