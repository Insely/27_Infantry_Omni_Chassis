/*
 * @Author: hao hao@qlu.edu.cn
 * @Date: 2025-09-02 20:32:38
 * @LastEditors: hao hao@qlu.edu.cn
 * @LastEditTime: 2025-09-02 23:30:31
 * @FilePath: \Season-26-Code\User\Hardware\LZ_motor_driver.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __LZ_MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#define __LZ_MOTOR_DRIVER_H

#include "main.h"
#include "stdint.h"
#include "fdcan.h"

// MIT 协议指令类型（高3位）
#define MIT_CMD_ENABLE     0x00
#define MIT_CMD_DISABLE    0x01
#define MIT_CMD_MIT_PARAM  0x02
#define MIT_CMD_SET_ZERO   0x03
#define MIT_CMD_CLEAR_FAULT 0x04
#define MIT_CMD_SET_MODE   0x05
#define MIT_CMD_SET_ID     0x06
#define MIT_CMD_SET_PROTOCOL 0x07
#define MIT_CMD_SET_MASTER_ID 0x08
#define MIT_CMD_POSITION   0x09
#define MIT_CMD_VELOCITY   0x0A

// 默认电机和主机ID
#define DEFAULT_MOTOR_ID   1
#define DEFAULT_MASTER_ID  0xFD

// 参数范围定义
#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -33.0f
#define V_MAX 33.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -14.0f
#define T_MAX 14.0f

// 电机控制模式
typedef enum {
    LZ_MODE_MIT = 0,    // MIT模式
    LZ_MODE_POSITION,   // 位置模式
    LZ_MODE_VELOCITY,   // 速度模式
    LZ_MODE_CURRENT,    // 电流模式
    LZ_MODE_DISABLE     // 失能模式
} LZ_Mode_t;

// 电机状态结构体
typedef struct {
    uint8_t motor_id;
    float angle;        // 角度 (rad)
    float velocity;     // 速度 (rad/s)
    float torque;       // 转矩 (N.m)
    float temperature;  // 温度 (°C)
    uint32_t fault;     // 故障码
    uint8_t uid[8];     // 唯一标识符
} LZ_Motor_State_t;

// 电机控制结构体
typedef struct {
    uint8_t id;
    uint8_t master_id;
    LZ_Mode_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float kp_set;
    float kd_set;
    float current_limit;
    LZ_Motor_State_t state;
} LZ_Motor_t;

// 函数声明
void lz_send_command(uint8_t can_bus, uint8_t motor_id, uint8_t cmd_type, uint8_t *data);
void lz_enable_motor(uint8_t can_bus, uint8_t motor_id);
void lz_disable_motor(uint8_t can_bus, uint8_t motor_id);
void lz_send_mit_params(uint8_t can_bus, uint8_t motor_id, float angle, float speed, float kp, float kd, float torque);
void lz_set_zero(uint8_t can_bus, uint8_t motor_id);
void lz_clear_fault(uint8_t can_bus, uint8_t motor_id);
void lz_set_mode(uint8_t can_bus, uint8_t motor_id, uint8_t mode);
void lz_set_id(uint8_t can_bus, uint8_t motor_id, uint8_t new_id);
void lz_set_protocol(uint8_t can_bus, uint8_t motor_id, uint8_t protocol);
void lz_set_master_id(uint8_t can_bus, uint8_t motor_id, uint8_t master_id);
void lz_set_position(uint8_t can_bus, uint8_t motor_id, float target_pos, float max_speed);
void lz_set_velocity(uint8_t can_bus, uint8_t motor_id, float target_vel, float current_limit);

// 辅助函数
uint16_t float_to_uint_LZ(float x, float x_min, float x_max, uint8_t bits);
float uint_to_float_LZ(uint16_t x, float x_min, float x_max, uint8_t bits);


#ifdef __cplusplus
}
#endif
#endif /* __MIT_MOTOR_H */