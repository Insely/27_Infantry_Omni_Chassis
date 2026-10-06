/**
 * @file BoardLink.c
 * @brief 云台板与底盘板之间的数据路由、周期调度及 CAN/RS485/UART 传输。
 *
 * @note 公共传输层由两块板共用；发送和接收内容按照 BOARD_GIMBAL、
 *       BOARD_CHASSIS 分别编译。板卡身份在 robot_param.h 中配置。
 */
#include "BoardLink.h"
#include "CAN_receive_send.h"


#include <math.h>
#include <string.h>

/*
 * RS485 与普通 UART 共用的串行物理帧（消息 ID 和 CRC 均按小端发送）：
 * A5 5A | 版本 | 序号 | 消息数量 | 负载长度 |
 * [消息 ID(2) | 数据长度(1) | 数据(N)] ... | CRC16-CCITT(2)
 *
 * 原有 8 字节 BoardLink 消息仍作为逻辑协议单元。每个 1 ms 调度周期内到期的
 * 逻辑消息会聚合到同一个串行物理帧中，并通过一次 DMA 发送。
 * 下列常量继续使用 RS485 前缀，以保持线协议命名兼容。
 */
#define BOARD_LINK_RS485_SOF_1             0xA5U
#define BOARD_LINK_RS485_SOF_2             0x5AU
#define BOARD_LINK_RS485_VERSION           0x02U
#define BOARD_LINK_RS485_HEADER_LEN        6U
#define BOARD_LINK_RS485_ITEM_HEADER_LEN   3U
#define BOARD_LINK_RS485_CRC_LEN           2U
#define BOARD_LINK_RS485_MAX_ITEMS         8U
#define BOARD_LINK_RS485_MAX_PAYLOAD       96U
#define BOARD_LINK_RS485_MAX_FRAME_LEN     (BOARD_LINK_RS485_HEADER_LEN + \
                                             BOARD_LINK_RS485_MAX_PAYLOAD + \
                                             BOARD_LINK_RS485_CRC_LEN)
#define BOARD_LINK_RS485_RX_GAP_TIMEOUT_MS 5U
#define BOARD_LINK_RS485_RESPONSE_TIMEOUT_MS 3U

typedef enum
{
    BOARD_LINK_REFEREE_SHOOT_DATA = 0x091,
    BOARD_LINK_REFEREE_AMMO_DATA  = 0x092,
    BOARD_LINK_CHASSIS_SPEED_XY   = 0x101,
    BOARD_LINK_CHASSIS_SPEED_W    = 0x102,
    BOARD_LINK_CHASSIS_MODE       = 0x104,
    BOARD_LINK_IMU_ATTITUDE       = 0x105,
    BOARD_LINK_IMU_GYRO           = 0x106,
    BOARD_LINK_YAW_FEEDBACK       = 0x107,
    BOARD_LINK_TRIGGER_MODE       = 0x108,
    BOARD_LINK_BODY_GYRO          = 0x109,
} BoardLinkMessageId_e;

typedef struct
{
    BoardLinkMessageId_e id;
    void (*pack)(uint8_t data[BOARD_LINK_FRAME_LEN]);
    uint8_t divider;
    uint8_t phase;
} BoardLinkTxFrame_t;

typedef struct
{
    BoardLinkMessageId_e id;
    void (*receive)(uint8_t data[BOARD_LINK_FRAME_LEN]);
} BoardLinkRxFrame_t;

volatile BoardLink_t BoardLink = { .control_mode = BL_LOCK };

static uint8_t BoardLink_HandleRxFrame(uint16_t id, const uint8_t *data, uint8_t len);

/* ======================================== 传输层选择 ======================================== */

#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) || \
    (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_UART)

static uint8_t rs485_tx_buffer[BOARD_LINK_RS485_MAX_FRAME_LEN];
static uint8_t rs485_rx_buffer[BOARD_LINK_RS485_MAX_FRAME_LEN];
static volatile uint8_t rs485_tx_busy;
static uint8_t rs485_tx_payload_len;
static uint8_t rs485_tx_item_count;
static uint8_t rs485_tx_sequence;
static uint8_t rs485_rx_count;
static uint8_t rs485_rx_expected_len;
static uint32_t rs485_rx_last_tick;
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485
static volatile uint8_t rs485_waiting_reply;
static volatile uint8_t rs485_reply_pending;
static uint32_t rs485_request_tick;
#endif

/**
 * @brief 获取当前串行板间通信使用的 UART 句柄。
 * @return 普通 UART 模式返回 UART1；RS485 模式返回 USART2/USART3。
 * @note 该选择在编译期完成，运行期不会切换串口。
 */
static UART_HandleTypeDef *BoardLink_SerialHandle(void)
{
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_UART
    return &huart1;
#elif BOARD_LINK_RS485_UART == 2U
    return &huart2;
#elif BOARD_LINK_RS485_UART == 3U
    return &huart3;
#else
    return NULL;
#endif
}

/**
 * @brief 计算 RS485/UART 共用串行帧的 CRC16-CCITT 校验值。
 * @param data 待校验数据的首地址。
 * @param len 参与校验的字节数，不包括帧尾 CRC 本身。
 * @return uint16_t CRC16，多项式 0x1021，初值 0xFFFF。
 * @note CRC 在线上按低字节、高字节顺序发送。
 */
static uint16_t BoardLink_Crc16Ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief 清空串行字节流解析进度。
 * @note 在初始化、帧校验完成、帧头错误或串口异常后调用。
 *       此函数只重置接收状态，不清空已解析到 BoardLink 的业务数据。
 */
static void BoardLink_SerialResetRx(void)
{
    rs485_rx_count = 0U;
    rs485_rx_expected_len = 0U;
}

/**
 * @brief 开始组装一个新的串行物理帧。
 * @return uint8_t 1=发送缓冲区可用且帧头已初始化；
 *         0=上一帧 DMA 仍在发送，或主机仍在等待底盘回复。
 * @note 该函数不立即启动 UART，后续由 BoardLink_SerialAppend()
 *       追加逻辑消息，再由 BoardLink_SerialCommitPacket() 一次性发送。
 */
static uint8_t BoardLink_SerialBeginPacket(void)
{
    if (rs485_tx_busy != 0U)
        return 0U;
#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) && BOARD_CHASSIS
    if (!rs485_reply_pending) return 0U;
#endif

#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) && BOARD_GIMBAL
    /* 云台作为主机：等待底盘回复，超时后才发起下一次请求。 */
    if (rs485_waiting_reply != 0U)
    {
        if ((HAL_GetTick() - rs485_request_tick) <= BOARD_LINK_RS485_RESPONSE_TIMEOUT_MS)
            return 0U;
        rs485_waiting_reply = 0U;
    }
#endif

    rs485_tx_buffer[0] = BOARD_LINK_RS485_SOF_1;
    rs485_tx_buffer[1] = BOARD_LINK_RS485_SOF_2;
    rs485_tx_buffer[2] = BOARD_LINK_RS485_VERSION;
    rs485_tx_buffer[3] = rs485_tx_sequence;
    rs485_tx_buffer[4] = 0U;
    rs485_tx_buffer[5] = 0U;
    rs485_tx_payload_len = 0U;
    rs485_tx_item_count = 0U;
    return 1U;
}

/**
 * @brief 将一条 BoardLink 逻辑消息追加到当前串行物理帧。
 * @param id 逻辑消息 ID，沿用原 CAN ID 数值，但在串行传输中不参与硬件仲裁。
 * @param data 逻辑消息负载。
 * @param len 负载长度，当前 BoardLink 消息通常为 8 字节。
 * @return uint8_t 1=追加成功；0=参数无效、消息数超限或物理帧容量不足。
 * @note 每条消息在物理帧中保存为 message_id(2B)+data_len(1B)+data(N)。
 */
static uint8_t BoardLink_SerialAppend(uint16_t id, const uint8_t *data, uint8_t len)
{
    uint16_t required = (uint16_t)BOARD_LINK_RS485_ITEM_HEADER_LEN + len;
    uint16_t offset;

    if (data == NULL || len == 0U ||
        rs485_tx_item_count >= BOARD_LINK_RS485_MAX_ITEMS ||
        ((uint16_t)rs485_tx_payload_len + required) > BOARD_LINK_RS485_MAX_PAYLOAD)
        return 0U;

    offset = (uint16_t)BOARD_LINK_RS485_HEADER_LEN + rs485_tx_payload_len;
    rs485_tx_buffer[offset++] = (uint8_t)(id & 0xFFU);
    rs485_tx_buffer[offset++] = (uint8_t)(id >> 8);
    rs485_tx_buffer[offset++] = len;
    memcpy(&rs485_tx_buffer[offset], data, len);

    rs485_tx_payload_len = (uint8_t)(rs485_tx_payload_len + required);
    rs485_tx_item_count++;
    rs485_tx_buffer[4] = rs485_tx_item_count;
    rs485_tx_buffer[5] = rs485_tx_payload_len;
    return 1U;
}

/**
 * @brief 完成当前串行物理帧，追加 CRC16 并启动 UART DMA 发送。
 * @return uint8_t 1=无数据需发送或 DMA 启动成功；0=UART 无效、忙或 HAL 发送失败。
 * @note rs485_tx_buffer 是 DMA 直接访问的静态缓冲区。发送期间由
 *       rs485_tx_busy 防止其被下一个调度周期改写，完成回调再解锁。
 */
static uint8_t BoardLink_SerialCommitPacket(void)
{
    UART_HandleTypeDef *huart = BoardLink_SerialHandle();
    uint16_t frame_len;
    uint16_t crc;

    if (rs485_tx_item_count == 0U)
        return 1U;
    if (huart == NULL || rs485_tx_busy != 0U)
        return 0U;

    frame_len = (uint16_t)BOARD_LINK_RS485_HEADER_LEN +
                rs485_tx_payload_len + BOARD_LINK_RS485_CRC_LEN;
    crc = BoardLink_Crc16Ccitt(rs485_tx_buffer,
                              (uint16_t)(frame_len - BOARD_LINK_RS485_CRC_LEN));
    rs485_tx_buffer[frame_len - 2U] = (uint8_t)(crc & 0xFFU);
    rs485_tx_buffer[frame_len - 1U] = (uint8_t)(crc >> 8);

    /* DMA 期间 tx_buffer 不得被改写，tx_busy 在完成回调中清零。 */
    rs485_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(huart, rs485_tx_buffer, frame_len) != HAL_OK)
    {
        rs485_tx_busy = 0U;
        return 0U;
    }

#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) && BOARD_CHASSIS
    rs485_reply_pending = 0U;
#endif
    rs485_tx_sequence++;
#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) && BOARD_GIMBAL
    rs485_waiting_reply = 1U;
    rs485_request_tick = HAL_GetTick();
#endif
    return 1U;
}

/**
 * @brief 校验并解析已完整接收的串行物理帧。
 * @return uint8_t 1=CRC、消息边界和总长度均正确；0=帧内容非法。
 * @note 函数会遍历帧中的所有逻辑消息，并通过
 *       BoardLink_HandleRxFrame() 复用与 CAN 相同的业务解包函数。
 *       至少解出一条有效底盘反馈后，解除主机等待状态。
 */
static uint8_t BoardLink_SerialParsePacket(void)
{
    uint16_t payload_end = (uint16_t)BOARD_LINK_RS485_HEADER_LEN + rs485_rx_buffer[5];
    uint16_t received_crc = (uint16_t)rs485_rx_buffer[payload_end] |
                            ((uint16_t)rs485_rx_buffer[payload_end + 1U] << 8);
    uint16_t calculated_crc = BoardLink_Crc16Ccitt(rs485_rx_buffer, payload_end);
    uint16_t offset = BOARD_LINK_RS485_HEADER_LEN;
    uint8_t handled = 0U;

    if (received_crc != calculated_crc)
        return 0U;

    uint16_t check = offset;
    for (uint8_t item = 0; item < rs485_rx_buffer[4]; ++item) {
        if (check + 3U > payload_end) return 0U;
        uint8_t size = rs485_rx_buffer[check + 2U];
        if (size != BOARD_LINK_FRAME_LEN || check + 3U + size > payload_end) return 0U;
        check += 3U + size;
    }
    if (check != payload_end) return 0U;
    for (uint8_t item = 0U; item < rs485_rx_buffer[4]; item++)
    {
        uint16_t id;
        uint8_t len;

        if ((offset + BOARD_LINK_RS485_ITEM_HEADER_LEN) > payload_end)
            return 0U;

        id = (uint16_t)rs485_rx_buffer[offset] |
             ((uint16_t)rs485_rx_buffer[offset + 1U] << 8);
        len = rs485_rx_buffer[offset + 2U];
        offset += BOARD_LINK_RS485_ITEM_HEADER_LEN;

        if ((offset + len) > payload_end)
            return 0U;

        if (BoardLink_HandleRxFrame(id, &rs485_rx_buffer[offset], len) == BOARD_LINK_RX_HANDLED)
            handled = 1U;
        offset += len;
    }

    if (offset != payload_end)
        return 0U;

#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) && BOARD_GIMBAL
    if (handled != 0U)
        rs485_waiting_reply = 0U;
#elif BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485
    if (handled) rs485_reply_pending = 1U;
#else
    (void)handled;
#endif
    return 1U;
}

/**
 * @brief 向共用串行解析器投递一段 UART DMA 接收数据。
 * @param data 本次 DMA/IDLE 回调收到的字节块。
 * @param len 字节块长度。
 * @note 支持半帧、粘包和任意字节对齐；解析器通过 A5 5A 帧头重新同步。
 *       如果两段数据的时间间隔超过限值，丢弃之前未完成的残帧。
 */
static void BoardLink_SerialFeed(const uint8_t *data, uint16_t len)
{
    uint32_t now = HAL_GetTick();

    if (data == NULL)
        return;

    if (rs485_rx_count != 0U &&
        (now - rs485_rx_last_tick) > BOARD_LINK_RS485_RX_GAP_TIMEOUT_MS)
        BoardLink_SerialResetRx();
    rs485_rx_last_tick = now;

    for (uint16_t i = 0U; i < len; i++)
    {
        uint8_t byte = data[i];

        if (rs485_rx_count == 0U)
        {
            if (byte == BOARD_LINK_RS485_SOF_1)
                rs485_rx_buffer[rs485_rx_count++] = byte;
            continue;
        }

        if (rs485_rx_count == 1U)
        {
            if (byte == BOARD_LINK_RS485_SOF_2)
                rs485_rx_buffer[rs485_rx_count++] = byte;
            else if (byte != BOARD_LINK_RS485_SOF_1)
                BoardLink_SerialResetRx();
            continue;
        }

        if (rs485_rx_count >= BOARD_LINK_RS485_MAX_FRAME_LEN)
        {
            BoardLink_SerialResetRx();
            continue;
        }

        rs485_rx_buffer[rs485_rx_count++] = byte;

        if (rs485_rx_count == BOARD_LINK_RS485_HEADER_LEN)
        {
            uint8_t payload_len = rs485_rx_buffer[5];

            if (rs485_rx_buffer[2] != BOARD_LINK_RS485_VERSION ||
                rs485_rx_buffer[4] == 0U ||
                rs485_rx_buffer[4] > BOARD_LINK_RS485_MAX_ITEMS ||
                payload_len > BOARD_LINK_RS485_MAX_PAYLOAD)
            {
                BoardLink_SerialResetRx();
                continue;
            }

            rs485_rx_expected_len = (uint8_t)(BOARD_LINK_RS485_HEADER_LEN +
                                              payload_len +
                                              BOARD_LINK_RS485_CRC_LEN);
        }

        if (rs485_rx_expected_len != 0U && rs485_rx_count == rs485_rx_expected_len)
        {
            BoardLink_SerialParsePacket();
            BoardLink_SerialResetRx();
        }
    }
}

#endif /* RS485 或普通 UART 串行传输 */

/**
 * @brief 初始化当前条件编译选中的 BoardLink 传输层。
 * @note CAN 模式不需要额外状态；串行模式会重置序号、DMA 忙标志和接收解析器。
 */
static void BoardLink_TransportInit(void)
{
#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) || \
    (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_UART)
    rs485_tx_busy = 0U;
    rs485_tx_sequence = 0U;
    rs485_rx_last_tick = 0U;
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485
    rs485_waiting_reply = 0U;
    rs485_reply_pending = 0U;
    rs485_request_tick = 0U;
#endif
    BoardLink_SerialResetRx();
#endif
}

/**
 * @brief 开始一次 BoardLink 调度周期的发送操作。
 * @return uint8_t 1=传输层可接收逻辑消息；0=串行 DMA 仍忙，本周期暂不发送。
 * @note CAN 模式下始终返回 1；RS485/UART 模式下会初始化聚合帧头。
 */
static uint8_t BoardLink_TransportBegin(void)
{
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_CAN
    return 1U;
#else
    return BoardLink_SerialBeginPacket();
#endif
}

/**
 * @brief 使用当前选中的传输方式提交一条逻辑消息。
 * @param id BoardLink 逻辑消息 ID。
 * @param data 负载数据。
 * @param len 负载长度。
 * @return uint8_t 1=CAN 已入队或串行消息已追加到聚合帧；0=提交失败。
 * @note CAN 模式立即调用 FDCAN；RS485/UART 模式只写入静态聚合缓冲区。
 */
static uint8_t BoardLink_TransportSend(uint16_t id, const uint8_t *data, uint8_t len)
{
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_CAN
    if (len != BOARD_LINK_FRAME_LEN)
        return 0U;
    return (Fdcanx_SendData(Get_CanHandle(BOARD_LINK_CAN_BUS - 1U),
                            id,
                            (uint8_t *)data,
                            len) == 0U) ? 1U : 0U;
#else
    return BoardLink_SerialAppend(id, data, len);
#endif
}

/**
 * @brief 结束一次 BoardLink 调度周期的发送操作。
 * @return uint8_t 1=提交成功；0=串行 DMA 未能启动。
 * @note CAN 模式的每条消息在 TransportSend 中已经发送；
 *       RS485/UART 模式在此处为整个聚合帧计算 CRC 并启动一次 DMA。
 */
static uint8_t BoardLink_TransportCommit(void)
{
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_CAN
    return 1U;
#else
    return BoardLink_SerialCommitPacket();
#endif
}

/* 线协议统一显式使用小端编码，避免未对齐访问和指针别名转换。 */
static void StoreFloat(float value, uint8_t *data)
{
    uint32_t bits; memcpy(&bits,&value,sizeof(bits));
    for (unsigned i=0;i<4;++i) data[i]=(uint8_t)(bits>>(8*i));
}
static float LoadFloat(uint8_t *data)
{
    uint32_t bits=(uint32_t)data[0] | ((uint32_t)data[1]<<8) |
                  ((uint32_t)data[2]<<16) | ((uint32_t)data[3]<<24);
    float value; memcpy(&value,&bits,sizeof(value)); return value;
}

/* ========================================== 云台板 ========================================== */
#if BOARD_GIMBAL

static uint8_t tx_tick;

static uint16_t LoadUint16(uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1]<<8);
}

static uint8_t LoadUint8(uint8_t *data)
{
    return data[0];
}

static void Pack_ChassisSpeedXY(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    StoreFloat(BoardLink.vx, &data[0]);
    StoreFloat(BoardLink.vy, &data[4]);
}

static void Pack_ChassisSpeedW(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    StoreFloat(BoardLink.w, &data[0]);
    StoreFloat(BoardLink.yaw_speed_cmd, &data[4]);
}

static void Pack_ChassisMode(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    data[0] = BoardLink.control_mode;
    data[1] = BoardLink.chassis_mode;
    data[2] = BoardLink.cap_mode;
    data[3] = BoardLink.reset;
    data[4] = BoardLink.ui_sequence;
    data[7] = 2U; /* 协议版本号，用于拒绝旧版浮点枚举格式。 */
}

static void Pack_ImuAttitude(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    StoreFloat(BoardLink.pitch, &data[0]);
    StoreFloat(BoardLink.yaw_cnt, &data[4]);
}

static void Pack_ImuGyro(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    StoreFloat(BoardLink.gyro[0], &data[0]);
    StoreFloat(BoardLink.gyro[2], &data[4]);
}

static void Pack_TriggerMode(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    data[0] = BoardLink.trigger_mode;
    data[1] = BoardLink.auto_mode;
    data[2] = BoardLink.shoot_mode;
    data[7] = 2U;
}

static void Receive_YawFeedback(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    float angle = LoadFloat(data), speed = LoadFloat(data + 4);
    if (!isfinite(angle) || !isfinite(speed)) return;
    BoardLink.yaw_angle_cnt = angle; BoardLink.yaw_spd = speed;
}

static void Receive_BodyGyro(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    float gyro = LoadFloat(data);
    if (!isfinite(gyro) || fabsf(gyro) > 40.0f) return;
    BoardLink.body_gyro_z = gyro;
}

static void Receive_RefereeShootData(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    BoardLink.initial_speed = LoadFloat(&data[0]);
    BoardLink.barrel_heat = LoadUint16(&data[4]);
    BoardLink.heat_limit = LoadUint16(&data[6]);
}

static void Receive_RefereeAmmoData(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    BoardLink.projectile_allowance_17mm = LoadUint16(&data[0]);
    BoardLink.launching_frequency = LoadUint8(&data[2]);
}

static const BoardLinkTxFrame_t tx_frames[] = {
    { BOARD_LINK_CHASSIS_SPEED_W,  Pack_ChassisSpeedW,  1, 0 },
    { BOARD_LINK_CHASSIS_MODE,     Pack_ChassisMode,    5, 1 },
    { BOARD_LINK_TRIGGER_MODE,     Pack_TriggerMode,    5, 3 },
    { BOARD_LINK_CHASSIS_SPEED_XY, Pack_ChassisSpeedXY, 2, 0 },
    { BOARD_LINK_IMU_ATTITUDE,     Pack_ImuAttitude,    4, 0 },
    { BOARD_LINK_IMU_GYRO,         Pack_ImuGyro,        4, 2 },
};

static const BoardLinkRxFrame_t rx_frames[] = {
    { BOARD_LINK_YAW_FEEDBACK,       Receive_YawFeedback },
    { BOARD_LINK_BODY_GYRO, Receive_BodyGyro },
    { BOARD_LINK_REFEREE_SHOOT_DATA, Receive_RefereeShootData },
    { BOARD_LINK_REFEREE_AMMO_DATA,  Receive_RefereeAmmoData },
};

static uint8_t fast_resend[sizeof(tx_frames) / sizeof(tx_frames[0])];
static uint32_t last_control_mode;
static uint32_t last_trigger_mode;

static uint32_t SampleControlMode(void)
{
    return ((uint32_t)BoardLink.control_mode << 24) | ((uint32_t)BoardLink.cap_mode << 20) | ((uint32_t)BoardLink.reset << 19) | ((uint32_t)BoardLink.ui_sequence << 8) | BoardLink.chassis_mode;
}

static uint32_t SampleTriggerMode(void)
{
    return ((uint32_t)BoardLink.trigger_mode << 16) | ((uint32_t)BoardLink.shoot_mode << 8) | BoardLink.auto_mode;
}

static void DetectModeChange(void)
{
    uint32_t control_mode = SampleControlMode();
    uint32_t trigger_mode = SampleTriggerMode();

    if (control_mode != last_control_mode)
    {
        last_control_mode = control_mode;
        BoardLink_RequestFastResend(BL_ID_CONTROL_MODE);
    }
    if (trigger_mode != last_trigger_mode)
    {
        last_trigger_mode = trigger_mode;
        BoardLink_RequestFastResend(BL_ID_TRIGGER_MODE);
    }
}

void BoardLink_Init(void)
{
    memset((void *)&BoardLink, 0, sizeof(BoardLink));
    BoardLink.control_mode = BL_LOCK;
    tx_tick = 0;
    memset(fast_resend, 0, sizeof(fast_resend));
    last_control_mode = SampleControlMode();
    last_trigger_mode = SampleTriggerMode();
    BoardLink_TransportInit();
}

void BoardLink_RequestFastResend(uint16_t message_id)
{
    for (uint8_t i = 0; i < sizeof(tx_frames) / sizeof(tx_frames[0]); i++)
    {
        if ((uint16_t)tx_frames[i].id == message_id)
        {
            fast_resend[i] = BOARD_LINK_FAST_RESEND_COUNT;
            return;
        }
    }
}

void BoardLink_TxStep(void)
{
    uint8_t data[BOARD_LINK_FRAME_LEN];
    uint32_t accepted_mask = 0U;

    DetectModeChange();

    if (!BoardLink_TransportBegin())
        goto advance_tick;

    for (uint8_t i = 0; i < sizeof(tx_frames) / sizeof(tx_frames[0]); i++)
    {
        uint8_t should_send = (tx_tick % tx_frames[i].divider == tx_frames[i].phase);

        if (fast_resend[i] > 0)
            should_send = 1;
        if (!should_send)
            continue;

        memset(data, 0, sizeof(data));
        tx_frames[i].pack(data);

        if (BoardLink_TransportSend((uint16_t)tx_frames[i].id,
                                    data,
                                    BOARD_LINK_FRAME_LEN))
            accepted_mask |= (1UL << i);
    }

    if (BoardLink_TransportCommit())
    {
        for (uint8_t i = 0; i < sizeof(tx_frames) / sizeof(tx_frames[0]); i++)
        {
            if ((accepted_mask & (1UL << i)) != 0U && fast_resend[i] > 0U)
                fast_resend[i]--;
        }
    }

advance_tick:
    tx_tick++;
    if (tx_tick >= 20U)
        tx_tick = 0;
}

/* ========================================== 底盘板 ========================================== */
#elif BOARD_CHASSIS

static uint8_t tx_tick;

static void StoreUint16(uint16_t value, uint8_t *data)
{
    data[0]=(uint8_t)value; data[1]=(uint8_t)(value>>8);
}

static void StoreUint8(uint8_t value, uint8_t *data)
{
    data[0]=value;
}

static void Pack_YawFeedback(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    StoreFloat(BoardLink.yaw_angle_cnt, &data[0]);
    StoreFloat(BoardLink.yaw_spd, &data[4]);
}

static void Pack_BodyGyro(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    StoreFloat(BoardLink.body_gyro_z, data);
}

static void Pack_RefereeShootData(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    StoreFloat(BoardLink.initial_speed, &data[0]);
    StoreUint16(BoardLink.barrel_heat, &data[4]);
    StoreUint16(BoardLink.heat_limit, &data[6]);
}

static void Pack_RefereeAmmoData(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    StoreUint16(BoardLink.projectile_allowance_17mm, &data[0]);
    StoreUint8(BoardLink.launching_frequency, &data[2]);
}

static void Receive_ChassisSpeedXY(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    float x = LoadFloat(data), y = LoadFloat(data + 4);
    if (!isfinite(x) || !isfinite(y) || fabsf(x) > 10.0f || fabsf(y) > 10.0f) return;
    BoardLink.vx = x; BoardLink.vy = y;
}

static void Receive_ChassisSpeedW(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    float w = LoadFloat(data), yaw = LoadFloat(data + 4);
    if (!isfinite(w) || !isfinite(yaw) || fabsf(w) > 20.0f || fabsf(yaw) > BOARD_LINK_YAW_SPEED_LIMIT_RAD_S) return;
    BoardLink.w = w; BoardLink.yaw_speed_cmd = yaw;
}

static void Receive_ChassisMode(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    if (data[7] != 2U || data[0] > BL_KEY ||
        (data[1] != BL_FLOW && data[1] != BL_SPIN_P && data[1] != BL_SPIN_N && data[1] != BL_NO_FOLLOW) ||
        data[2] > 1U || data[3] > 1U) return;
    BoardLink.control_mode = data[0]; BoardLink.chassis_mode = data[1];
    BoardLink.cap_mode = data[2]; BoardLink.reset = data[3]; BoardLink.ui_sequence = data[4];
}

static void Receive_ImuAttitude(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    BoardLink.pitch = LoadFloat(&data[0]);
    BoardLink.yaw_cnt = LoadFloat(&data[4]);
}

static void Receive_ImuGyro(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    BoardLink.gyro[0] = LoadFloat(&data[0]);
    BoardLink.gyro[2] = LoadFloat(&data[4]);
}

static void Receive_TriggerMode(uint8_t data[BOARD_LINK_FRAME_LEN])
{
    if (data[7] != 2U ||
        (data[0] != BL_CLOSE && data[0] != BL_HIGH && data[0] != BL_SINGLE && data[0] != BL_DEBUG) ||
        data[1] > 1U || data[2] > 2U) return;
    BoardLink.trigger_mode = data[0]; BoardLink.auto_mode = data[1]; BoardLink.shoot_mode = data[2];
}

static const BoardLinkTxFrame_t tx_frames[] = {
    { BOARD_LINK_YAW_FEEDBACK,       Pack_YawFeedback,       1,  0 },
    { BOARD_LINK_BODY_GYRO, Pack_BodyGyro, 1, 0 },
    { BOARD_LINK_REFEREE_SHOOT_DATA, Pack_RefereeShootData, 20,  2 },
    { BOARD_LINK_REFEREE_AMMO_DATA,  Pack_RefereeAmmoData,  20,  9 },
};

static const BoardLinkRxFrame_t rx_frames[] = {
    { BOARD_LINK_CHASSIS_SPEED_XY, Receive_ChassisSpeedXY },
    { BOARD_LINK_CHASSIS_SPEED_W,  Receive_ChassisSpeedW },
    { BOARD_LINK_CHASSIS_MODE,     Receive_ChassisMode },
    { BOARD_LINK_IMU_ATTITUDE,     Receive_ImuAttitude },
    { BOARD_LINK_IMU_GYRO,         Receive_ImuGyro },
    { BOARD_LINK_TRIGGER_MODE,     Receive_TriggerMode },
};

void BoardLink_Init(void)
{
    memset((void *)&BoardLink, 0, sizeof(BoardLink));
    BoardLink.control_mode = BL_LOCK;
    tx_tick = 0;
    BoardLink.yaw_speed_cmd = 0.0f;
    BoardLink_TransportInit();
}

void BoardLink_RequestFastResend(uint16_t message_id)
{
    (void)message_id;
}

void BoardLink_TxStep(void)
{
    uint8_t data[BOARD_LINK_FRAME_LEN];

    if (!BoardLink_TransportBegin())
        goto advance_tick;

    for (uint8_t i = 0; i < sizeof(tx_frames) / sizeof(tx_frames[0]); i++)
    {
        if (tx_tick % tx_frames[i].divider != tx_frames[i].phase)
            continue;

        memset(data, 0, sizeof(data));
        tx_frames[i].pack(data);
        BoardLink_TransportSend((uint16_t)tx_frames[i].id,
                                data,
                                BOARD_LINK_FRAME_LEN);
    }

    BoardLink_TransportCommit();

advance_tick:
    tx_tick++;
    if (tx_tick >= 20U)
        tx_tick = 0;
}

#else
#error "BoardLink 必须且只能选择一个板卡身份"
#endif

/* ======================================== 公共接收入口 ====================================== */

/**
 * @brief 将一条已通过传输层校验的逻辑消息分发给对应解包函数。
 * @param id BoardLink 逻辑消息 ID。
 * @param data 逻辑负载首地址。
 * @param len 逻辑负载长度，当前必须为 BOARD_LINK_FRAME_LEN。
 * @return uint8_t BOARD_LINK_RX_HANDLED=找到对应 ID 并已解包；否则返回未处理。
 * @note CAN、RS485 和普通 UART 共用该入口，因此业务解包不依赖底层传输方式。
 */
static uint8_t BoardLink_HandleRxFrame(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len != BOARD_LINK_FRAME_LEN)
        return BOARD_LINK_RX_UNHANDLED;

    for (uint8_t i = 0; i < sizeof(rx_frames) / sizeof(rx_frames[0]); i++)
    {
        if ((uint16_t)rx_frames[i].id == id)
        {
            rx_frames[i].receive((uint8_t *)data);
            return BOARD_LINK_RX_HANDLED;
        }
    }
    return BOARD_LINK_RX_UNHANDLED;
}

/**
 * @brief CAN 接收回调的 BoardLink 分发入口。
 * @param hfdcan 产生接收中断的 FDCAN 句柄。
 * @param id CAN 标准帧 ID，同时作为 BoardLink 逻辑消息 ID。
 * @param data 8 字节 CAN 负载。
 * @return uint8_t BOARD_LINK_RX_HANDLED=已处理；BOARD_LINK_RX_UNHANDLED=不属于 BoardLink。
 * @note 只有编译为 CAN 传输时才会消费帧；串行模式始终返回未处理。
 */
uint8_t BoardLink_RxDispatch(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t data[8])
{
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_CAN
    if (hfdcan != Get_CanHandle(BOARD_LINK_CAN_BUS - 1U))
        return BOARD_LINK_RX_UNHANDLED;
    return BoardLink_HandleRxFrame(id, data, BOARD_LINK_FRAME_LEN);
#else
    (void)hfdcan;
    (void)id;
    (void)data;
    return BOARD_LINK_RX_UNHANDLED;
#endif
}

/**
 * @brief UART IDLE/DMA 接收回调的 BoardLink 串行分发入口。
 * @param huart 产生回调的 UART 句柄。
 * @param data 本次接收到的字节块。
 * @param len 字节块长度。
 * @return uint8_t BOARD_LINK_RX_HANDLED=该 UART 是当前板间串行端口；否则返回未处理。
 * @note 返回已处理仅表示该字节块已交给解析器，不代表其中一定含有完整合法帧。
 */
uint8_t BoardLink_UartRxDispatch(UART_HandleTypeDef *huart,
                                 const uint8_t *data,
                                 uint16_t len)
{
#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) || \
    (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_UART)
    if (huart != BoardLink_SerialHandle())
        return BOARD_LINK_RX_UNHANDLED;

    BoardLink_SerialFeed(data, len);
    return BOARD_LINK_RX_HANDLED;
#else
    (void)huart;
    (void)data;
    (void)len;
    return BOARD_LINK_RX_UNHANDLED;
#endif
}

/**
 * @brief 处理 UART DMA 发送完成事件，释放串行发送缓冲区。
 * @param huart 完成发送的 UART 句柄。
 * @note 由 HAL_UART_TxCpltCallback() 调用；非 BoardLink 串口不会改变任何状态。
 */
void BoardLink_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) || \
    (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_UART)
    if (huart == BoardLink_SerialHandle())
        rs485_tx_busy = 0U;
#else
    (void)huart;
#endif
}

/**
 * @brief 处理 BoardLink 串行 UART 错误，解锁发送并丢弃当前残帧。
 * @param huart 发生错误的 UART 句柄。
 * @note 该函数只重置 BoardLink 内部状态；RX DMA 的重启仍由 UART BSP 错误回调完成。
 */
void BoardLink_UartErrorCallback(UART_HandleTypeDef *huart)
{
#if (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485) || \
    (BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_UART)
    if (huart == BoardLink_SerialHandle())
    {
        rs485_tx_busy = 0U;
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_RS485
        rs485_waiting_reply = 0U;
#endif
        BoardLink_SerialResetRx();
    }
#else
    (void)huart;
#endif
}
