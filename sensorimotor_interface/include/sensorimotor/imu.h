#pragma once
#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1)

/**
 * IMU data structure.
 */
typedef struct
{
    // Acceleration. uinit: m/s2
	float acc_x;
	float acc_y;
	float acc_z;

    // Angular velocity. uinit: rad/s
	float gyro_x;
	float gyro_y;
	float gyro_z;

    // Orientation. uinit: rad
	float pitch;
	float roll;
	float yaw;
	
    // Orientation in quaternion
	float quaternion_w;
	float quaternion_x;	
	float quaternion_y;
	float quaternion_z;
} imu_data_t;

#pragma pack()

/**
 * IMU data callback function type.
 * @param data  IMU data
 * @param param Pointer for using in class
 */
typedef void (*imu_data_callback_t)(const imu_data_t *data, void *param);

/**
 * Initialize IMU device.
 * @param device_path   IMU device path.
 * @param calibrate     Calibrate IMU orientation.
 * @param callback      Output IMU data.
 * @return 0 for success, others for failure.
 */
int initialize_imu(const char *device_path, bool calibrate=false, 
	imu_data_callback_t callback=NULL, void *callback_param=NULL);

/**
 * Deinitialize IMU device.
 */
void deinitialize_imu();

/**
 * Get the current IMU data.
 * @param data   IMU data.
 * @return 0 for success, others for failure.
 */
int get_imu_data(imu_data_t *data);


#ifdef __cplusplus
}
#endif

