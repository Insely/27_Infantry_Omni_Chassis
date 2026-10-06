#include "Shoot.h"
#include "Global_status.h"
#include "Gimbal.h"
#include "referee_system.h"
#include "Stm32_time.h"
#include "User_math.h"

Shoot_t Shoot;
static double trigger_angle_target = 0;   //目标总角度
static uint32_t last_shoot_time = 0;      //上次射击时间戳

/*----------------------------------- 初始化 -----------------------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Shoot_Init()
{
	// 摩擦轮电机初始化
	
	
	// 拨弹电机初始化
	TriggerMotor_init(DJI_M2006, TRIGGER_MOTOR);

	// 拨弹电机
	PID_Set(&Shoot.trigger_speed_pid, 5, 0, 3, SHOOTMOTOR_MAX_CURRENT, 0);
	PID_Set(&Shoot.trigger_location_pid, 5, 0, 0, SHOOTMOTOR_MAX_CURRENT, 0);

	//初始化角度目标
	trigger_angle_target = 0;
}


/*--------------------------------- 状态量更新 ---------------------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */
void Shoot_Updater()
{
	/*------状态量更新------*/
	//速度
	Shoot.trigger_speed_now = (TriggerMotor_get_data(TRIGGER_MOTOR).speed_rpm);
	//电流
	Shoot.trigger_current_now = (TriggerMotor_get_data(TRIGGER_MOTOR).given_current);
    //获取累计角度
	Shoot.trigger_angle_now = (TriggerMotor_get_data(TRIGGER_MOTOR).angle_cnt);

	/*------目标量更新------*/
    //摩擦轮
	static enum shoot_mode_e pre_shoot_mode = CLOSE;

// 模式切换时的启动逻辑
if (pre_shoot_mode == CLOSE && Global.Shoot.shoot_mode != CLOSE) {
    // 从关闭状态启动
    Shoot.shoot_speed_set = SHOOT_SPEED_BEGIN;
    Global.Shoot.shoot_status = NOK;
    pre_shoot_mode = Global.Shoot.shoot_mode;
} 
else {
    // 正常工作状态
    switch(Global.Shoot.shoot_mode) {
        case CLOSE:
            Shoot.shoot_speed_set = SHOOT_SPEED_CLOSE;
            Global.Shoot.shoot_status = NOK;
            break;
        case READY:
            Shoot.shoot_speed_set = SHOOT_SPEED_READY;
            // 达速检测
            // if ((abs(Shoot.shoot_speed_L_now - SHOOT_SPEED_READY) < 300) &&
            //     (abs(Shoot.shoot_speed_R_now - SHOOT_SPEED_READY) < 300)) {
                Global.Shoot.shoot_status = OK;
            //}
            break;
        case DEBUG_SHOOT:
            Shoot.shoot_speed_set = SHOOT_SPEED_BEGIN;
            Global.Shoot.shoot_status = OK;
            break;
        default:
            Shoot.shoot_speed_set = SHOOT_SPEED_CLOSE;
            Global.Shoot.shoot_status = NOK;
            break;
    }
    pre_shoot_mode = Global.Shoot.shoot_mode;
}
    uint32_t current_time = Get_SysTime_ms();
	// 拨弹电机
	//根据模式选择速度
	switch (Global.Shoot.trigger_mode)
	{
	case HIGH:
		Shoot.trigger_speed_set = TRIGGER_SPEED_HIGH;
		trigger_angle_target = Shoot.trigger_angle_now;
		break;
	case DEBUG_TRIGGER:
		Shoot.trigger_speed_set = TRIGGER_SPEED_DEBUG;
		trigger_angle_target = Shoot.trigger_angle_now;
		break;
	case SINGLE:
	    if(current_time - last_shoot_time > SINGLE_FIRE_INTERVAL)
		{
			trigger_angle_target += TRIGGER_ANGLE_INC;
			last_shoot_time = current_time;
		}
		break;
	case TRIGGER_CLOSE:
	default:
		Shoot.trigger_speed_set = TRIGGER_SPEED_CLOSE;
		trigger_angle_target = Shoot.trigger_angle_now;
		break;
	}
}


/*----------------------------------- 解算 -------------------------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
void Shoot_Calculater()
{
	// // 热量控制
 	//  if (Referee_data.Barrel_Heat_17mm < (Referee_data.Heat_Limit - 50)  && Shoot.trigger_speed_set != TRIGGER_SPEED_CLOSE) // 150
	// {
	// 	if (Referee_data.Barrel_Heat_17mm < (Referee_data.Heat_Limit - 100)) // 100
	// 		Shoot.trigger_speed_set = TRIGGER_SPEED_MID;
	// 	else if ((Referee_data.Barrel_Heat_17mm > (Referee_data.Heat_Limit - 100)) && (Referee_data.Barrel_Heat_17mm < (Referee_data.Heat_Limit - 90))) // 80--110
	// 		Shoot.trigger_speed_set = TRIGGER_SPEED_LOW;
	// }
	// else
	// 	Shoot.trigger_speed_set = TRIGGER_SPEED_CLOSE;  

	// 卡弹处理
	static int trigger_kill_cnt = 0; // 卡弹回退次数
	if (abs(Shoot.trigger_current_now) > 10000)
		trigger_kill_cnt = 50; // 更改此值可以改变回退幅度
	if (trigger_kill_cnt > 1)
	{
		Shoot.trigger_speed_set = -10000;
		trigger_kill_cnt--;
		trigger_angle_target = Shoot.trigger_angle_now;
	}
	else
	{
        if(Global.Shoot.trigger_mode == SINGLE)
	    {
		    float speed_set_from_pos = PID_Cal(&Shoot.trigger_location_pid,(float)Shoot.trigger_angle_now,(float)trigger_angle_target);
		    Shoot.trigger_speed_set = speed_set_from_pos;
	    }
	}
	Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid, Shoot.trigger_speed_now, Shoot.trigger_speed_set);
}


/*----------------------------------- 控制 -------------------------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Shoot_Controller()
{
    if (Global.Control.mode == LOCK) {
        SHOOTMotor_set(0, TRIGGER_MOTOR);
        return;
    }

	SHOOTMotor_set(Shoot.current[2],TRIGGER_MOTOR);
}


/*----------------------------------- 任务 -------------------------------------*/

/**
 * @brief          发射机构任务
 * @param          none
 * @retval         none
 */
void Shoot_Tasks(void)
{
#if (USE_SHOOT != 0)
	// 发射机构数据更新
	Shoot_Updater();
	// 发射机构控制计算
	Shoot_Calculater();
	// 发射机构电机控制
	Shoot_Controller();

#endif
}
