/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-30 20:47:26
 * @FilePath: \Season-26-Code\User\Hardware\Remote\VT13.c
 */
#include "VT13.h"

VT13_data_t VT13_data;

/**
 * @brief          遥控器协议解析
 * @param[in]      VT13_buf: 原生数据指针
 * @param[out]     rc_ctrl: 遥控器数据指
 * @retval         none
 */
void VT13_DataSolve(volatile const uint8_t *VT13_buf, VT13_data_t *rc_ctrl)
{
    if (VT13_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
		if(VT13_buf[0] == 0xa9 && VT13_buf[1] == 0x53) 
		{			
			rc_ctrl->rc.ch[0] = (VT13_buf[2] | (VT13_buf[3] << 8)) & 0x07ff;        //!< Channel 0
			rc_ctrl->rc.ch[1] = ((VT13_buf[3] >> 3) | (VT13_buf[4] << 5)) & 0x07ff; //!< Channel 1
			rc_ctrl->rc.ch[2] = ((VT13_buf[4] >> 6) | (VT13_buf[5] << 2) |          //!< Channel 2
													 (VT13_buf[6] << 10)) & 0x07ff;
			rc_ctrl->rc.ch[3] = ((VT13_buf[6] >> 1) | (VT13_buf[7] << 7)) & 0x07ff; //!< Channel 3
			rc_ctrl->rc.mode_sw = ((VT13_buf[7] >> 4) & 0x0003); 
			rc_ctrl->rc.stop = ((VT13_buf[7] >> 6) & 0x01);
			rc_ctrl->rc.left_button = ((VT13_buf[7] >> 7) & 0x01);//fn
			rc_ctrl->rc.right_button = ((VT13_buf[8] >> 0) & 0x01);
			rc_ctrl->rc.wheel = ((VT13_buf[8] >> 1) | (VT13_buf[9] << 7)) & 0x07FF;
			rc_ctrl->rc.shutter = (VT13_buf[9] >> 4) & 0x01;//扳机
			
			rc_ctrl->mouse.x = (VT13_buf[10] | (VT13_buf[11] << 8));
			rc_ctrl->mouse.y = (VT13_buf[12] | (VT13_buf[13] << 8));
			rc_ctrl->mouse.z = (VT13_buf[14] | (VT13_buf[15] << 8));
			
			rc_ctrl->mouse.press_l = (VT13_buf[16] >> 0) & 0x03;
			rc_ctrl->mouse.press_r = (VT13_buf[16] >> 2) & 0x03;
			rc_ctrl->mouse.middle = (VT13_buf[16] >> 4) & 0x03;
			
			rc_ctrl->key.v = (VT13_buf[17] | (VT13_buf[18] << 8));
			
			rc_ctrl->crc16 = (VT13_buf[19] | (VT13_buf[20] << 8));
			
			rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
			rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
			rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
			rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
			rc_ctrl->rc.wheel -= RC_CH_VALUE_OFFSET;
            rc_ctrl->online = 100;
		}
   
}
