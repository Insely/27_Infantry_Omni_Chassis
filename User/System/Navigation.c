/* 26_Sentry navigation wire layout and checksums, without navigation control. */
#include "Navigation.h"
#include "BoardLink.h"
#include "CRC8_CRC16.h"
#include "UART_data_txrx.h"
#include <string.h>
Navigation_data_t Navigation_receive_1;
STM32ROS_data_t stm32send_1;
int Navigation_online;
_Static_assert(sizeof(Navigation_data_t)==54, "Navigation wire size");
_Static_assert(sizeof(STM32ROS_data_t)==23, "Navigation telemetry size");
uint8_t Navigation_DecodeData(Navigation_data_t *target, unsigned char buff[], unsigned int len)
{
    if (len != sizeof(Navigation_data_t) || buff[0] != 0xAA) {
        return 0; // 解析失败
    }

    uint16_t received_crc = (uint16_t)buff[len - 2] | ((uint16_t)buff[len - 1] << 8);
    uint16_t calculated_crc = Get_Modbus_CRC16(buff, len - 2);

    if (calculated_crc == received_crc) 
    {
        memcpy(target, buff, sizeof(Navigation_data_t));
        Navigation_online = 1;
        return 1; // 解析成功
    }
    
    return 0; // CRC 校验失败
}

/**
 * @brief 向导航发送裁判系统数据
 *
 */
void Navigation_SendMessage()
{
    /* Keep the active DMA buffer intact until the previous frame completes. */
    if (UART7_data.huart->gState != HAL_UART_STATE_READY) return;

    stm32send_1.remain_hp = 0;                                                   // 双板精简后不再转发
    stm32send_1.max_hp = 0;                                                      // 双板精简后不再转发
    stm32send_1.game_type = 0;                                                   // 双板精简后不再转发
    stm32send_1.game_progress = 0;                                               // 双板精简后不再转发
    stm32send_1.stage_remain_time = 0;                                           // 双板精简后不再转发
    stm32send_1.bullet_remaining_num_17mm = BoardLink.projectile_allowance_17mm; // 允许发弹量
    stm32send_1.outpost_hp = 0;                                                  // 双板精简后不再转发
    stm32send_1.base_hp = 0;                                                     // 双板精简后不再转发
    stm32send_1.rfid_status = 0;                                                 // 双板精简后不再转发
    stm32send_1.contact_angle = BoardLink.yaw_angle_cnt;
    stm32send_1.is_fire = BoardLink.trigger_mode;
    //stm32send_1.distance = fromMINIPC.distance;

     static uint8_t s_buff[sizeof(STM32ROS_data_t) + 3];
    s_buff[0] = 0xA5;
    memcpy(&s_buff[1], &stm32send_1, sizeof(STM32ROS_data_t));
    append_CRC16_check_sum(s_buff, sizeof(s_buff));

    /* Vircom_Send(s_buff, sizeof(s_buff)); */
    HAL_UART_Transmit_DMA(UART7_data.huart, s_buff, sizeof(s_buff));
    
}

/* Assemble the same 54-byte frame across arbitrary DMA fragments. */
void Navigation_RxBytes(const uint8_t *data, uint16_t len)
{
    static uint8_t buf[sizeof(Navigation_data_t)];
    static uint16_t used;
    for (uint16_t i=0;i<len;++i) {
        if(used==0 && data[i]!=0xAA) continue;
        buf[used++]=data[i];
        if(used==sizeof(buf)) {
            if(Navigation_DecodeData(&Navigation_receive_1,buf,sizeof(buf))) { used=0; Navigation_online=200; }
            else {
                memmove(buf,buf+1,--used);
                while(used && buf[0]!=0xAA) memmove(buf,buf+1,--used);
            }
        }
    }
}
