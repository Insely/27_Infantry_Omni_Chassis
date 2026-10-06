#ifndef __MOTOR_LZ_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __MOTOR_LZ_H__

#include "struct_typedef.h"

#include "CAN_receive_send.h"

#define motor_LZ_N 2
#define motor_LZ_user_id 0
#define LZ_CAN  hfdcan1

void Motor_LZ_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);

enum motorModel{
	MOTOR_LZ_00=0,
	MOTOR_LZ_01,
	MOTOR_LZ_02,
	MOTOR_LZ_03,
	MOTOR_LZ_04,
	MOTOR_LZ_05,
	MOTOR_LZ_06,
	MOTOR_LZ_EDU_05,
};

enum canComMode{
	CANCOM_ANNOUNCE_DEVID = 0,//通告设备 ID
	CANCOM_MOTOR_CTRL, //MOTOR-电机控制
	CANCOM_MOTOR_FEEDBACK, //MOTOR-电机反馈
	CANCOM_MOTOR_IN, //MOTOR-进入电机模式
	CANCOM_MOTOR_RESET, //MOTOR-复位模式
	CANCOM_MOTOR_CALI, //MOTOR-高速编码器标定
	CANCOM_MOTOR_ZERO, //MOTOR-设置机械零位
	CANCOM_MOTOR_ID, //MOTOR-设置 ID
	CANCOM_PARA_WRITE, //参数-写入
	CANCOM_PARA_READ, //参数-读取
	CANCOM_CALI_ING, //编码器标定中
	CANCOM_CALI_RST, //编码器标定结果
	CANCOM_PARA_STR_INFO, //参数-字符串信息
	CANCOM_MOTOR_BRAKE, //MOTOR-进入刹车模式
	CANCOM_FAULT_WARN, //故障和警告信息
	CANCOM_MODE_TOTAL, //电机数据保存帧
	CANCOM_MODE_Bd,		//电机波特率修改帧
	CANCOM_MODE_ACTIVE_RECV = 0x18,//电机主动上报帧
	CANCOM_MODE_AGREEMENT,//电机协议修改帧
};


typedef struct __pack
{
    uint8_t id    		 :5;      // 电机ID
    uint8_t model		 :3;
	
} Motor_LZ_Init;

typedef struct 
{
    uint16_t id    		 :8 ;      // 电机ID
    uint16_t data  		 :16;      // 数据
    uint8_t mode :5 ;      // 模式
	uint8_t  res		 :3 ;	   // 标志位
	
} RIS_Mode_t_LZ;

typedef struct {
	RIS_Mode_t_LZ exld;
	uint8_t len;		//数据长度
	uint8_t Data[8]; 	//数据
}canPack_LZ;

// t_ref=Kd*(v_set-v_actual)+Kp*(p_set-p_actual)+t_ff
typedef struct{
	enum canComMode mode;
	fp32 T;
	fp32 Angle;
	fp32 W;	
	fp32 Kp;
	fp32 Kd;
}user_send_Lz;

typedef struct{

	uint8_t MError;
	uint8_t mode;		//Reset 0  Cali 1  Motor 2;
	fp32 T;
	fp32 Angle;
	fp32 W;
	fp32 Temp;
}user_recv_Lz;


typedef struct{
	canPack_LZ    DataHanding;
	user_send_Lz  *Data_send;
	user_recv_Lz  Data_recv;
	uint8_t id;
	uint8_t enable;				//使能
	uint8_t model;
}motor_LZ;


/**
  * @brief          灵足电机初始化
  * @param[in]      none
  * @retval         none
  */
void motor_LZ_init();
/**
  * @brief          灵足电机外部控制参数获取
  * @param[in]      id 电机id
  * @param[in]    	*Data_send 数据指针
  * @retval         none
  */
void motor_LZ_send_init(uint8_t id,user_send_Lz  *Data_send);
/**
  * @brief          灵足电机返回数据获取
  * @param[in]      id 电机id
  * @retval         user_recv_Lz 数据指针
  */
const user_recv_Lz *motor_LZ_recv_return(uint8_t id);
/**
  * @brief          灵足电机控制数据处理
  * @param[in]      id 电机id
  * @retval         none
  */
void motor_LZ_send(uint8_t id);

/**
  * @brief          灵足电机使能
  * @param[in]      id 电机id
  * @retval         none
  */
void motor_LZ_enable(uint8_t id);

/**
  * @brief          灵足电机失能
  * @param[in]      id 电机id
  * @retval         none
  */
void motor_LZ_lose(uint8_t id);
/**
  * @brief          灵足电机主动上报帧
  * @param[in]      id 电机id
  * @param[in]		F_CMD模式更改 00关 01开启
  * @retval         none
  */
void motor_LZ_active_recv(uint8_t id,uint8_t F_CMD);

/**
  * @brief          灵足电机设置机械零点 会把当前电机位置设为机械零位， 会先失能电机, 再使能电机
  * @param[in]      id 电机id
  * @retval         none
  */
void motor_LZ_zero(uint8_t id);
/**
  * @brief          灵足电机设置CAN_ID  失能电机
  * @param[in]      id 电机id
  * @param[in] 		修改后（预设）CANID
  * @retval         none
  */
void motor_LZ_set_CAN_ID(uint8_t id,uint8_t set_id);


#ifdef __cplusplus
}
#endif
#endif