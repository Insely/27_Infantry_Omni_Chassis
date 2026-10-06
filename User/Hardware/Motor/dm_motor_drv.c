
#include "dm_motor_drv.h"
#include "CAN_receive_send.h"
#include "User_math.h"

#define abs(a) a > 0 ? a : -a

/**
************************************************************************
* @brief:      	dm4310_enable: 使能 DM4310 电机控制模式
* @param[in]:   hcan:    指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor:   指向 DM_motor_t 结构体的指针，包含电机状态和控制参数
* @retval:     	void
* @details:    	根据电机控制模式，通过 CAN 总线发送对应的使能命令。
*               支持 MIT、位置、速度和位置-速度-电流模式。
************************************************************************
**/
void dm_motor_enable(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			enable_motor_mode(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			enable_motor_mode(hcan, motor->id, PSI_MODE);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_disable: 失能 DM4310 电机控制模式
* @param[in]:   hcan:    指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor:   指向 DM_motor_t 结构体的指针，包含电机状态和控制参数
* @retval:     	void
* @details:    	根据电机控制模式，通过 CAN 总线发送对应的失能命令，随后清零控制参数。
*               支持 MIT、位置、速度和位置-速度-电流模式。
************************************************************************
**/
void dm_motor_disable(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			disable_motor_mode(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			disable_motor_mode(hcan, motor->id, PSI_MODE);
			break;
	}	
	dm_motor_clear_para(motor);
}
/**
************************************************************************
* @brief:      	dm4310_ctrl_send: 发送 DM4310 电机控制命令
* @param[in]:   hcan:    指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor:   指向 DM_motor_t 结构体的指针，包含电机状态和控制参数
* @retval:     	void
* @details:    	根据当前控制模式，向 DM4310 发送相应的控制帧。
*               支持 MIT、位置、速度和位置-速度-电流模式。
************************************************************************
**/
void dm_motor_ctrl_send(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case pos_mode:
			pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case spd_mode:
			spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
		case psi_mode:
			psi_ctrl(hcan, motor->id,motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
			break;
	}	
}

/**
************************************************************************
* @brief:      	dm4310_clear: 清零 DM4310 电机控制参数
* @param[in]:   motor:   指向 DM_motor_t 结构体的指针
* @retval:     	void
* @details:    	将位置、速度、比例增益、微分增益、转矩和电流设定值清零。
************************************************************************
**/
void dm_motor_clear_para(DM_motor_t *motor)
{
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
	motor->ctrl.cur_set = 0;
}
/**
************************************************************************
* @brief:      	dm4310_clear_err: 清除 DM4310 电机错误
* @param[in]:   hcan: 	 指向 CAN 控制结构体的指针
* @param[in]:  	motor:   指向电机结构体的指针
* @retval:     	void
* @details:    	根据当前控制模式，发送对应模式的清错命令。
************************************************************************
**/
void dm_motor_clear_err(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			clear_err(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			clear_err(hcan, motor->id, PSI_MODE);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_fbdata: 解析 DM4310 电机反馈数据
* @param[in]:   motor:    指向 DM_motor_t 结构体的指针
* @param[in]:   rx_data:  指向反馈数据数组的指针
* @retval:     	void
* @details:    	解析电机 ID、状态、位置、速度、转矩、MOS 温度和线圈温度，
*               同时累计圈数并计算相对角度。
************************************************************************
**/
void dm_motor_fbdata(DM_motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.last_pos = motor->para.pos;
	motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);

	// process the data
    // count cnt
    if (motor->para.last_pos > 12 && motor->para.pos < -12)
        motor->para.pos_cnt += ((P_MAX - motor->para.last_pos) + (motor->para.pos + P_MAX));
    else if (motor->para.last_pos < -12 && motor->para.pos > 12)
        motor->para.pos_cnt -= ((P_MAX - motor->para.pos) + motor->para.last_pos + P_MAX);
    else
        motor->para.pos_cnt += (motor->para.pos - motor->para.last_pos);
    // process data
     motor->para.angle_cnt = motor->para.pos_cnt * POS_TO_ANGLE;
    
        // 计算出轴转速
        //(ptr)->round_speed = (ptr)->speed_rpm;

    // 计算相对角度 -180~180 谨防精度丢失 总角度过大时
        float angle = motor->para.angle_cnt - motor->angle_zero;
        uint32_t mul = abs((int)angle) / 180;
        if (angle > 180.0f)
        {
            if (mul % 2 == 1) // 处于-180度
                angle -= (mul + 1) * 180;
            else // 处于180度
                angle -= mul * 180;
        }
        if (angle < -180.0f)
        {
            if (mul % 2 == 1) // 处于180度
                angle += (mul + 1) * 180;
            else // 处于-180度
                angle += mul * 180;
        }
        motor->para.angle = angle;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: 使能指定电机模式
* @param[in]:   hcan:     指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor_id: 目标电机 ID
* @param[in]:   mode_id:  要使能的模式 ID
* @retval:     	void
* @details:    	通过 CAN 总线向目标电机发送模式使能命令。
************************************************************************
**/
void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: 失能指定电机模式
* @param[in]:   hcan:     指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor_id: 目标电机 ID
* @param[in]:   mode_id:  要失能的模式 ID
* @retval:     	void
* @details:    	通过 CAN 总线向目标电机发送模式失能命令。
************************************************************************
**/
void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	save_pos_zero: 保存当前位置为零点
* @param[in]:   hcan:     指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor_id: 目标电机 ID
* @param[in]:   mode_id:  目标模式 ID
* @retval:     	void
* @details:    	通过 CAN 总线向目标电机发送保存零点命令。
************************************************************************
**/
void save_pos_zero(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	clear_err: 清除电机错误
* @param[in]:   hcan:     指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor_id: 目标电机 ID
* @param[in]:   mode_id:  目标模式 ID
* @retval:     	void
* @details:    	通过 CAN 总线向目标电机发送清错命令。
************************************************************************
**/
void clear_err(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	mit_ctrl: MIT 模式电机控制
* @param[in]:   hcan:			指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor_id:	目标电机 ID
* @param[in]:   pos:			位置设定值
* @param[in]:   vel:			速度设定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩设定值
* @retval:     	void
* @details:    	将控制量编码后，通过 CAN 总线发送 MIT 模式控制帧。
************************************************************************
**/
void mit_ctrl(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	pos_ctrl: 位置-速度模式控制
* @param[in]:   hcan:			指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor_id:	目标电机 ID
* @param[in]:   pos:			位置设定值
* @param[in]:   vel:			速度设定值
* @retval:     	void
* @details:    	通过 CAN 总线发送位置-速度控制命令。
************************************************************************
**/
void pos_ctrl(FDCAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	spd_ctrl: 速度模式控制
* @param[in]:   hcan: 		指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor_id: 目标电机 ID
* @param[in]:   vel: 			速度设定值
* @retval:     	void
* @details:    	通过 CAN 总线发送速度控制命令。
************************************************************************
**/
void spd_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPD_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	Fdcanx_SendData(hcan, id, data, 4);
}

/**
************************************************************************
* @brief:      	psi_ctrl: 位置、速度、电流混合控制
* @param[in]:   hcan:			指向 CAN_HandleTypeDef 结构体的指针
* @param[in]:   motor_id:	目标电机 ID
* @param[in]:   pos:			位置设定值
* @param[in]:   vel:			速度设定值
* @param[in]:   cur:			电流设定值
* @retval:     	void
* @details:    	通过 CAN 总线发送位置、速度和电流混合控制命令。
************************************************************************
**/
void psi_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float cur)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf, *ibuf;
	uint8_t data[8];
	
	uint16_t u16_vel = vel*100;
	uint16_t u16_cur  = cur*10000;
	
	id = motor_id + PSI_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&u16_vel;
	ibuf=(uint8_t*)&u16_cur;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	
	data[6] = *ibuf;
	data[7] = *(ibuf+1);
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	read_motor_data: 读取电机寄存器
* @param[in]:   id:    电机 CAN ID
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	发送读取电机寄存器命令。
************************************************************************
**/
void read_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	Fdcanx_SendData(&hfdcan1, 0x7FF, data, 4);
}
/**
************************************************************************
* @brief:      	read_motor_ctrl_fbdata: 读取电机控制反馈数据
* @param[in]:   id:    电机 CAN ID
* @retval:     	void
* @details:    	发送读取电机控制反馈数据命令。
************************************************************************
**/
void read_motor_ctrl_fbdata(uint16_t id) 
{
	uint8_t can_id_l = id & 0xFF;       // 低 8 位
	    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位

	uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
	Fdcanx_SendData(&hfdcan1, 0x7FF, data, 4);
}
/**
************************************************************************
* @brief:      	write_motor_data: 写入电机寄存器
* @param[in]:   id:    电机 CAN ID
* @param[in]:   rid:   寄存器地址
* @param[in]:   d0-d3: 写入的数据
* @retval:     	void
* @details:    	发送写入电机寄存器命令。
************************************************************************
**/
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
	uint8_t can_id_l = id & 0xFF;       // 低 8 位
	    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
	
	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
	Fdcanx_SendData(&hfdcan1, 0x7FF, data, 8);
}
/**
************************************************************************
* @brief:      	save_motor_data: 保存电机参数
* @param[in]:   id:    电机 CAN ID
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	发送保存电机参数命令。
************************************************************************
**/
void save_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0xFF;       // 低 8 位
	    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
	
	uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
	Fdcanx_SendData(&hfdcan1, 0x7FF, data, 4);
}
