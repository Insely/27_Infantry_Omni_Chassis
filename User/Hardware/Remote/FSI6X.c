#include "FSI6X.h"

FSI6X_data_Struct FSI6X_data;
void FSI6X_decode_data(uint8_t *sbus_buf,FSI6X_data_Struct *FSI6X_data)
{       
        if (sbus_buf == NULL || FSI6X_data == NULL)
        {
                return;
        }
        
    // 【关键修改】检查 SBUS 协议的 Byte 23 (状态字节)
    // Bit 3 (0x08) 为 1 表示失控保护激活 (Failsafe Activated)
    // Bit 2 (0x04) 为 1 表示丢帧 (Frame Lost)
    
    uint8_t status_byte = sbus_buf[23];

    if ((status_byte & 0x08) == 0x08) 
    {
        // === 检测到严重失控（遥控器关闭或断连） ===
        
        FSI6X_data->ConnectState = 0; // 标记为断开连接
        
        // 【安全措施】强制将所有控制通道归零或设为安全值
        FSI6X_data->CH1 = 0; 
        FSI6X_data->CH2 = 0;
        FSI6X_data->CH3 = 0; // 假设这是油门，必须归零
        FSI6X_data->CH4 = 0;
        FSI6X_data->CH5 = 240;
        FSI6X_data->CH6 = 240;
        FSI6X_data->CH7 = 240;
        FSI6X_data->CH8 = 240;
        
        // 可以在这里加一个全局急停标志
        // Global_Emergency_Stop = 1; 
        
        return; // 直接返回，不再解析错误的通道数据
    }
    else
    {
        // === 连接正常 ===
        FSI6X_data->ConnectState = 1;
        FSI6X_data->online = 30; // 喂狗/更新在线计数器
    }


        //FSI6X_data->ConnectState = 1;       
        FSI6X_data->CH1 =  (((uint16_t)sbus_buf[1] >> 0 | ((uint16_t)sbus_buf[2] << 8 )) & 0x07FF);//264 1024 1807
        FSI6X_data->CH2 =  (((uint16_t)sbus_buf[2] >> 3 | ((uint16_t)sbus_buf[3] << 5 )) & 0x07FF);//240 1024 1807
        FSI6X_data->CH3 =  (((uint16_t)sbus_buf[3] >> 6 | ((uint16_t)sbus_buf[4] << 2 ) | (int16_t)sbus_buf[5] << 10 ) & 0x07FF);//240 1024 1807
        FSI6X_data->CH4 =  (((uint16_t)sbus_buf[5] >> 1 | ((uint16_t)sbus_buf[6] << 7 )) & 0x07FF);//240  1024 1019 1807
        FSI6X_data->CH5 =  (((int16_t)sbus_buf[6] >> 4 | ((int16_t)sbus_buf[7] << 4 )) & 0x07FF);//240 1807
        FSI6X_data->CH6 =  (((int16_t)sbus_buf[7] >> 7 | ((int16_t)sbus_buf[8] << 1 ) | (int16_t)sbus_buf[9] << 9 ) & 0x07FF);//240 1807
        FSI6X_data->CH7 =  (((int16_t)sbus_buf[9] >> 2 | ((int16_t)sbus_buf[10] << 6 )) & 0x07FF);//240 1024 1807
        FSI6X_data->CH8 =  (((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[11] << 3 )) & 0x07FF);//240 1807
        FSI6X_data->CH9 =  (((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF);//240 1807
        FSI6X_data->CH10 = (((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF);//240 1807
        FSI6X_data->CH11 = (((int16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 ) | (int16_t)sbus_buf[16] << 10 ) & 0x07FF);
        FSI6X_data->CH12 = (((int16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF);
        FSI6X_data->CH13 = (((int16_t)sbus_buf[17] >> 4 | ((int16_t)sbus_buf[18] << 4 )) & 0x07FF);
        FSI6X_data->CH14 = (((int16_t)sbus_buf[18] >> 7 | ((int16_t)sbus_buf[19] << 1 ) | (int16_t)sbus_buf[20] << 9 ) & 0x07FF);
        FSI6X_data->CH15 = (((int16_t)sbus_buf[20] >> 2 | ((int16_t)sbus_buf[21] << 6 )) & 0x07FF);
        FSI6X_data->CH16 = (((int16_t)sbus_buf[21] >> 5 | ((int16_t)sbus_buf[22] << 3 )) & 0x07FF);
   

        FSI6X_data->CH1 -= FS_MID;
        FSI6X_data->CH2 -= FS_MID;
        FSI6X_data->CH3 -= FS_MID;
        FSI6X_data->CH4 -= FS_MID;
        FSI6X_data->online = 30;
}