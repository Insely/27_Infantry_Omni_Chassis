/**
 * @file IMU_Updata.h
 * @author sethome
 * @brief
 * @version 0.1
 * @date 2022-11-19
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "struct_typedef.h"
#include "Kalman_Filter_c.h"
#include "hipnuc_dec.h"
#include "dm_imu.h"

#ifndef __IMU_UPDATA__
#define __IMU_UPDATA__


struct IMU_t
{
	fp32 gyro[3];  // rad/s
	fp32 accel[3]; // m/s^2
	fp32 mag[3];   // 磁力计ut
	fp32 temp;	   // 温度
	
	float calibration[3];//角速度校准error
	float cali_end[3];

	Attitude_3D_t KF; //卡尔曼滤波结果

	struct
	{
		fp32 q[4];			   //四元数
		fp32 pitch, yaw, roll; //欧拉角
	} Mahony;

	struct
	{
		fp32 q[4];			   //四元数
		fp32 pitch, yaw, roll; //欧拉角
	} madgwick;

	struct
	{
		fp32 q[4];			   //四元数
		fp32 pitch, yaw, roll; //欧拉角
		int8_t err_code;
		fp32 last_yaw;
		fp32 yaw_rad_cnt;
		fp32 yaw_angle_cnt;
	} AHRS;

};

extern struct IMU_t IMU_data;
extern struct IMU_t IMU_data_history[10]; // IMU历史数据结构体
extern hipnuc_raw_t IMU_HI_GIMBAL_data;
extern unsigned  long ulTdleCycleCount;	


// 外部调用
void IMU_Init(void);   // IMU初始化
void IMU_Updata(void); // IMU数据更新
void MagUpdate(void);  // 磁力计数据更新
void MagZero(void); // 清零地磁计
void Get_Angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);//非调用Lib版本
void Process_IMUData();
float get_history_data(uint8_t yaw_or_pitch,uint32_t history_time);
float get_history_q_data(uint8_t i,uint32_t history_time);//获取历史数据函数
float rad2degree(float a);
float degree2rad(float a);
void IMU_CailProgram(void);
void IMU_Offest(void);
#endif
// end of flie
