#ifndef HIPNUC_CAN_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#define HIPNUC_CAN_DRIVER_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
    uint64_t hw_ts_us;
} hipnuc_can_frame_t;

#define HIPNUC_CAN_EFF_FLAG    0x80000000U
#define HIPNUC_CAN_SFF_MASK    0x000007FFU
#define HIPNUC_CAN_EFF_MASK    0x1FFFFFFFU

#define CAN_MSG_ERROR       0
#define CAN_MSG_ACCEL       1
#define CAN_MSG_GYRO        2
#define CAN_MSG_MAG         3
#define CAN_MSG_TEMP        4
#define CAN_MSG_QUAT        5
#define CAN_MSG_EULER       6
#define CAN_MSG_PRESSURE    7
#define CAN_MSG_GNSS_POS    8
#define CAN_MSG_GNSS_VEL    9
#define CAN_MSG_INCLI       10
#define CAN_MSG_TIME        11
#define CAN_MSG_PITCH_ROLL  12
#define CAN_MSG_YAW         13
#define CAN_MSG_GNSS_STATUS 14
#define CAN_MSG_UNKNOWN     99

typedef struct {
    uint8_t node_id;        // 节点ID
    uint64_t hw_ts_us;      // 硬件时间戳 (us)
    float acc_x;            // 加速度计 X轴 (G)
    float acc_y;            // 加速度计 Y轴 (G)
    float acc_z;            // 加速度计 Z轴 (G)
    float gyr_x;            // 陀螺仪 X轴 (deg/s)
    float gyr_y;            // 陀螺仪 Y轴 (deg/s)
    float gyr_z;            // 陀螺仪 Z轴 (deg/s)
    float mag_x;            // 磁力计 X轴 (uT)
    float mag_y;            // 磁力计 Y轴 (uT)
    float mag_z;            // 磁力计 Z轴 (uT)
    float quat_w;           // 四元数 W
    float quat_x;           // 四元数 X
    float quat_y;           // 四元数 Y
    float quat_z;           // 四元数 Z
    float roll;             // 横滚角 (deg)
    float pitch;            // 俯仰角 (deg)
    float imu_yaw;          // 偏航角 (deg)
    float incli_x;          // 倾角 X
    float incli_y;          // 倾角 Y
    float temperature;      // 温度 (℃)
    float pressure;         // 气压 (Pa)
    uint8_t utc_year;       // UTC 年
    uint8_t utc_month;      // UTC 月
    uint8_t utc_day;        // UTC 日
    uint8_t hours;          // 时
    uint8_t minutes;        // 分
    uint8_t seconds;        // 秒
    uint16_t milliseconds;  // 毫秒
    uint32_t timestamp_ms;  // 时间戳 (ms)
    double ins_lat;         // 纬度 (deg)
    double ins_lon;         // 经度 (deg)
    double ins_msl;         // 海拔高度 (m)
    float undulation;       // 大地水准面差距
    float diff_age_s;       // 差分龄期 (s)
    float ins_vel_e;        // 东向速度 (m/s)
    float ins_vel_n;        // 北向速度 (m/s)
    float ins_vel_u;        // 天向(垂直)速度 (m/s)
    float ins_speed;        // 合速度 (m/s)
    uint8_t solq_pos;       // 位置解状态
    uint8_t solq_heading;   // 定向解状态
    uint8_t nv_pos;         // 定位卫星数
    uint8_t nv_heading;     // 定向卫星数
    uint8_t ins_status;     // 组合导航状态
} can_sensor_data_t;

typedef struct {
    char buffer[512];
    size_t length;
} can_json_output_t;

int hipnuc_can_to_json(const can_sensor_data_t *data, int msg_type, can_json_output_t *output);
uint8_t hipnuc_can_extract_node_id(uint32_t can_id);

typedef enum {
    HIPNUC_J1939_CMD_READ = 0x03,
    HIPNUC_J1939_CMD_WRITE = 0x06
} hipnuc_j1939_cmd_t;

void hipnuc_j1939_build_reg_write_cmd(uint8_t da, uint8_t sa, uint16_t addr, uint32_t val, hipnuc_can_frame_t *out);
void hipnuc_j1939_build_reg_read_cmd(uint8_t da, uint8_t sa, uint16_t addr, hipnuc_can_frame_t *out);
int hipnuc_j1939_parse_cmd(const hipnuc_can_frame_t *frame, uint16_t *addr, hipnuc_j1939_cmd_t *cmd, uint8_t *status, uint32_t *val);

int hipnuc_j1939_is_cfg_frame(const hipnuc_can_frame_t *frame);
void hipnuc_j1939_build_sync(uint8_t da, uint8_t sa, uint32_t pgn, hipnuc_can_frame_t *out);



#ifdef __cplusplus
}
#endif
#endif
