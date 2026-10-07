#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <stdint.h>
#pragma pack(push, 1)
typedef struct
{
    uint8_t game_type;                    // 比赛类型
    uint8_t game_progress;                // 比赛阶段 (0未开始, 4进行中, 5结算)
    uint16_t remain_hp;                   // 当前血量
    uint16_t max_hp;                      // 最大血量
    uint16_t stage_remain_time;           // 当前阶段剩余时间 (s)
    uint16_t bullet_remaining_num_17mm;   // 17mm弹丸剩余允许发射量
    uint16_t outpost_hp;                  // 己方前哨站血量
    uint16_t base_hp;                     // 己方基地血量
    uint32_t rfid_status;                 // RFID状态位 (各bit对应不同增益点)
    float contact_angle;                  // 单 Yaw 相对角 (deg)
    uint8_t is_fire;                      // 是否开火 (0=不开火, 非0=开火)
} STM32ROS_data_t;


/**
 * @brief ROS导航系统 -> STM32 下发数据 (底盘运动指令)
 * @note  帧格式: [header][payload][crc16], 通过UART接收
 */

typedef struct
{
    uint8_t header;              // 帧头 (固定值)
    float x_speed;               // x方向速度指令 (m/s, 机体坐标系)
    float y_speed;               // y方向速度指令 (m/s, 机体坐标系)
    float rotate;                // 旋转速度指令 (rad/s)
    float yaw_speed;             // yaw轴期望角速度 (rad/s)
    uint8_t running_state;       // 运动姿态切换 (0=正常, 1=加速冲刺...)
    uint8_t region_code;         // 指定区域编号 (11=小陀螺区域)
    uint8_t yaw_aligned;     // Yaw 是否已对齐 (1=对齐)
    float reserved[8];           // 预留字段
    uint16_t crc_val;            // CRC16校验 (Modbus)
} Navigation_data_t;



#pragma pack(pop)
extern Navigation_data_t Navigation_receive_1;
extern STM32ROS_data_t stm32send_1;
extern int Navigation_online;
uint8_t Navigation_DecodeData(Navigation_data_t *target, unsigned char buff[], unsigned int len);
void Navigation_RxBytes(const uint8_t *data, uint16_t len);
void Navigation_SendMessage(void);
#endif
