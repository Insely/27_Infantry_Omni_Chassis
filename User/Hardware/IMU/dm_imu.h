/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-12-25 19:37:17
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2025-12-27 08:37:56
 * @FilePath: \Regular_Sentry_Gimbal\User\Hardware\IMU\dm_imu.h
 */

#ifndef __DM_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#define __DM_IMU_H

#include "stm32h7xx_hal.h"



#define ACCEL_CAN_MAX (235.2f)
#define ACCEL_CAN_MIN	(-235.2f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#define CMD_READ 0
#define CMD_WRITE 1

typedef enum
{
	COM_USB=0,
	COM_RS485,
	COM_CAN,
	COM_VOFA

}imu_com_port_e;

typedef enum
{
	CAN_BAUD_1M=0,
	CAN_BAUD_500K,
	CAN_BAUD_400K,
	CAN_BAUD_250K,
	CAN_BAUD_200K,
	CAN_BAUD_100K,
	CAN_BAUD_50K,
	CAN_BAUD_25K
	
}imu_baudrate_e;

typedef enum 
{
	REBOOT_IMU=0,
	ACCEL_DATA,
	GYRO_DATA,
	EULER_DATA,
	QUAT_DATA,
	SET_ZERO,
	ACCEL_CALI,
	GYRO_CALI,
	MAG_CALI,
	CHANGE_COM,
	SET_DELAY,
	CHANGE_ACTIVE,
	SET_BAUD,
	SET_CAN_ID,
	SET_MST_ID,
	DATA_OUTPUT_SELECTION,
	SAVE_PARAM=254,
	RESTORE_SETTING=255
}reg_id_e;



typedef struct
{
	uint8_t can_id;
	uint8_t mst_id;
	
	FDCAN_HandleTypeDef *can_handle;
	
	float pitch;//旋转坐标系
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;

	float last_yaw;
	float yaw_cnt;
	
	uint32_t request_cnt;


}imu_t;
extern imu_t dm_imu_gimbal;
void imu_init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan,imu_t *imu);
void imu_write_reg(uint8_t reg_id,uint32_t data,imu_t *imu);
void imu_read_reg(uint8_t reg_id,imu_t *imu);
void imu_reboot(imu_t *imu);
void imu_accel_calibration(imu_t *imu);
void imu_gyro_calibration(imu_t *imu);
void imu_change_com_port(imu_com_port_e port,imu_t *imu);
void imu_set_active_mode_delay(uint32_t delay,imu_t *imu);
void imu_change_to_active(imu_t *imu);
void imu_change_to_request(imu_t *imu);
void imu_set_baud(imu_baudrate_e baud,imu_t *imu);
void imu_set_can_id(uint8_t can_id,imu_t *imu);
void imu_set_mst_id(uint8_t mst_id,imu_t *imu);
void imu_save_parameters(imu_t *imu);
void imu_restore_settings(imu_t *imu);
void imu_request_accel(imu_t *imu);
void imu_request_gyro(imu_t *imu);
void imu_request_euler(imu_t *imu);
void imu_request_quat(imu_t *imu);
void IMU_Rotate_Frame(float angle_deg,imu_t *imu);
void IMU_MatchData(imu_t *imu);
void IMU_UpdateData(uint8_t* pData,imu_t *imu);


#ifdef __cplusplus
}
#endif
#endif
