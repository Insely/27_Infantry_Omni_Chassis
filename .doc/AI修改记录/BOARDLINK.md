# BoardLink v2

CAN 默认总线 CAN2；所有逻辑负载 8 字节。float 使用 IEEE-754 binary32，小端。保留的空字节必须填零。应用层枚举不得直接替代下列线协议枚举。

| ID | 方向 | 内容 | 调度 |
| --- | --- | --- | --- |
| 0x101 | G→C | float vx, float vy（m/s） | 2 ms |
| 0x102 | G→C | float w, float yaw_speed_cmd（rad/s） | 1 ms |
| 0x104 | G→C | uint8 control、chassis、cap、reset、ui_sequence、0、0、version=2 | 5 ms / 变化后快速重发 |
| 0x105 | G→C | float pitch, float yaw（degree，监测数据） | 4 ms |
| 0x106 | G→C | float gyro_x, float gyro_z（rad/s，监测数据） | 4 ms |
| 0x108 | G→C | uint8 trigger、auto、shoot、0、0、0、0、version=2 | 5 ms / 变化后快速重发 |
| 0x107 | C→G | float Yaw 相对位置（degree）, float 电机速度（degree/s） | 1 ms |
| 0x109 | C→G | float 底盘 gyro_z（rad/s）, 4 个零字节 | 1 ms |
| 0x091 | C→G | float 初速度, uint16 热量, uint16 热量上限 | 20 ms |
| 0x092 | C→G | uint16 允许弹量, uint8 发射频率, 5 个零字节 | 20 ms |

control：RC=0、LOCK=1、KEY=2。chassis：FLOW=0、SPIN_P=1、SPIN_N=2、NO_FOLLOW=4；3 为保留的导航值，本版不执行。trigger：CLOSE=0、HIGH=1、SINGLE=4、DEBUG=5。shoot：CLOSE=0、READY=1、DEBUG=2。auto：NONE=0、CAR=1。cap：0/1。ui_sequence 为重绘事件计数器。

Yaw 命令最大绝对值 20 rad/s；XY 最大绝对值 10 m/s，w 最大绝对值 20 rad/s。非法浮点/枚举使对应命令组失效。完整四组控制数据分别在 20 ms 内有效才允许底盘执行；Yaw 相对位置超时 8 ms、底盘 gyro 超时 20 ms 时云台停止下发非零 Yaw 速度。

串行封装：`A5 5A | version=2 | sequence | item_count | payload_len | [id:u16, len:u8, data] ... | crc:u16`。CRC16-CCITT，多项式 0x1021，初值 0xFFFF，CRC 低字节在前。先校验全部 TLV 边界再应用消息。sequence 目前用于观察帧序号，不是可靠传输的确认号；不承诺丢帧重传，状态通过周期发送和模式变化后的 5 次快速重发恢复。

RS485 从机只响应合法完整的主机请求；主机等待应答超过 3 ms 后允许再次发送。串行接收片段间隔超过 5 ms 时丢弃残帧。串口 DMA 忙时不改写缓冲区。

这是本版双板内部协议，不可与旧 Infantry 的 0x101～0x10B 遥控转发协议或原 fold 协议混刷。切换通信类型需同时更新两板的公共配置及物理接线。
