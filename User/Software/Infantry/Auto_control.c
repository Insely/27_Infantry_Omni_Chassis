#include "Auto_control.h"
#include "Global_status.h"
#include "Gimbal.h"

#include "IMU_updata.h"
#include "referee_system.h"
#include "UART_data_txrx.h"

#include "CRC8_CRC16.h"
#include "string.h"
#include "USB_VirCom.h"
#include "robot_param.h"

STM32_data_t toMINIPC;
MINIPC_data_t fromMINIPC;

uint8_t data[128];
uint8_t rx_data[100];

void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len)
{
    if (len == sizeof(*target)) memcpy(target, buff, len);
}

int encodeSTM32(STM32_data_t *target, unsigned char tx_buff[], unsigned int len)
{
    memcpy(tx_buff, target, len);
    return 0;
}

void STM32_to_MINIPC()
{
    toMINIPC.header[0] = 'S';
    toMINIPC.header[1] = 'P';
    toMINIPC.mode = MODE_AUTO_AIM;
    toMINIPC.yaw = degree2rad(dm_imu_gimbal.yaw);//IMU_data.AHRS.yaw;
    toMINIPC.pitch = degree2rad(-dm_imu_gimbal.pitch);//IMU_data.AHRS.pitch;
    toMINIPC.yaw_vel = (cos(dm_imu_gimbal.pitch * DEG_TO_RAD) * dm_imu_gimbal.gyro[2] - sin(dm_imu_gimbal.pitch * DEG_TO_RAD) * dm_imu_gimbal.gyro[0]);//IMU_data.gyro[2];
    toMINIPC.pitch_vel = -dm_imu_gimbal.gyro[1]; //IMU_data.gyro[0];
    toMINIPC.q[0] = dm_imu_gimbal.q[0];//IMU_data.AHRS.q[0];
    toMINIPC.q[1] = dm_imu_gimbal.q[1];//IMU_data.AHRS.q[1];
    toMINIPC.q[2] = dm_imu_gimbal.q[2];//IMU_data.AHRS.q[2];
    toMINIPC.q[3] = dm_imu_gimbal.q[3];//IMU_data.AHRS.q[3];
    toMINIPC.bullet_speed = Referee_data.Initial_SPEED;
    toMINIPC.bullet_count = Referee_data.Launching_Frequency;
    int len = (uint8_t *)&toMINIPC.crc16 - (uint8_t *)&toMINIPC;
    // 此时调用 CRC，无论结构体怎么变都永远正确
    toMINIPC.crc16 = get_CRC16_check_sum(toMINIPC.header, len, 0xFFFF);
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_UART
    Vircom_Send(data,sizeof(STM32_data_t));
#else
    UART_SendData(UART1_data, data, sizeof(STM32_data_t));
#endif   //实体串口
}

void MINIPC_to_STM32(void)
{
    float yaw_error;   // in degrees

    // 差分滤波状态
    static float    last_shoot_yaw   = 0;
    static float    last_shoot_pitch = 0;
    static uint8_t  target_init  = 0;   
    static uint16_t reject_count = 0;   
    static uint16_t zero_count   = 0;  

    // 滤波调节参数
    const float    YAW_JUMP_MAX_DEG = 12.0f;  
    const float    PITCH_JUMP_MAX   = 0.26f;  
    const uint16_t REJECT_MAX       = 50;     
    const uint16_t ZERO_MAX         = 10;     

    Global.Auto.input.control_mode = fromMINIPC.mode;

    if (fromMINIPC.pitch == 0 && fromMINIPC.yaw == 0)
    {
        zero_count++;
        if (zero_count >= ZERO_MAX)
            Global.Auto.input.control_mode = CTRL_NO_CONTROL;
    }
    else
    {
        zero_count = 0;
    }

    if (Global.Auto.input.control_mode != CTRL_NO_CONTROL)
    {
        // 计算误差(弧度) 并立刻转换为绝对目标位置(度)
        uint8_t is_zero_frame = (fromMINIPC.pitch == 0 && fromMINIPC.yaw == 0);

        if (!is_zero_frame)
        {
            yaw_error = rad2degree(fromMINIPC.yaw) - dm_imu_gimbal.yaw;
            if (yaw_error > 180.0f)  yaw_error -= 360.0f;
            if (yaw_error < -180.0f) yaw_error += 360.0f;
        // 锁定收到数据这一刻的绝对目标位置，后续Auto_Control直接使用，不再每周期叠加
            float new_shoot_yaw   = Gimbal.yaw_location_now + yaw_error;
            float new_shoot_pitch = fromMINIPC.pitch;

            if (!target_init)   
            {
                last_shoot_yaw   = new_shoot_yaw;
                last_shoot_pitch = new_shoot_pitch;
                target_init = 1;
            }

            float d_yaw   = fabsf(new_shoot_yaw   - last_shoot_yaw);
            float d_pitch = fabsf(new_shoot_pitch - last_shoot_pitch);

            if ((d_yaw <= YAW_JUMP_MAX_DEG && d_pitch <= PITCH_JUMP_MAX) || reject_count >= REJECT_MAX)
            {
                Global.Auto.input.shoot_yaw   = new_shoot_yaw;
                Global.Auto.input.shoot_pitch = new_shoot_pitch;
                last_shoot_yaw   = new_shoot_yaw;
                last_shoot_pitch = new_shoot_pitch;
                reject_count = 0;

                Global.Auto.input.yaw_ff       = fromMINIPC.yaw_vel;
                Global.Auto.input.pitch_ff     = fromMINIPC.pitch_vel;
                Global.Auto.input.yaw_acc_ff   = fromMINIPC.yaw_acc;
                Global.Auto.input.pitch_acc_ff = fromMINIPC.pitch_acc;
            }
            else
            {
                reject_count++;  
            }
        }
        else
        {
            reject_count++;      
        }
    }
    else
    {
        target_init  = 0;
        reject_count = 0;
    }
}

void Auto_Control()
{
    if (Global.Auto.input.control_mode != CTRL_NO_CONTROL)
    {
        // 直接使用MINIPC_to_STM32中计算好的绝对目标位置
        Gimbal_SetYawAngle(Global.Auto.input.shoot_yaw);
        Gimbal_SetPitchAngle(Global.Auto.input.shoot_pitch);
    }
}
