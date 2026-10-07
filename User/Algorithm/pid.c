/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-12 10:37:00
 * @FilePath: \Regular_Sentry_Chassis\User\Algorithm\pid.c
 */
#include "math.h"
#include "pid.h"

/*
 p_set: 比例系数, i_set: 积分系数, d_set: 微分系数, f_set: 前馈系数
 lim_out_set: 总输出限制, lim_i_outset: 积分输出限制
 */
void PID_Set(pid_t *PidSet, float p_set, float i_set, float d_set, float f_set, float lim_out_set, float lim_i_outset)
{
  PidSet->p = p_set;
  PidSet->i = i_set;
  PidSet->d = d_set;
  PidSet->f = f_set;

  PidSet->lim_out = lim_out_set;
  PidSet->lim_i_out = lim_i_outset; // 将设置赋值
}

// PID计算
float PID_Cal(pid_t *PidGoal, float Now, float Set) // PID 计算
{
  PidGoal->set = Set;
  PidGoal->err_last = PidGoal->err;
  PidGoal->err = Set - Now; // 计算误差
  PidGoal->diff = PidGoal->err - PidGoal->err_last;

  PidGoal->p_out = PidGoal->p * PidGoal->err;
  if (PidGoal->i != 0)
    PidGoal->i_out += PidGoal->i * PidGoal->err;
  PidGoal->d_out = PidGoal->d * PidGoal->diff; // pid运算
  PidGoal->f_out = PidGoal->f * Set;           // 前馈运算

  if (fabs(PidGoal->i_out) > PidGoal->lim_i_out) // 防止积分过大
  {
    if (PidGoal->i_out < 0)
      PidGoal->i_out = -PidGoal->lim_i_out;
    else
      PidGoal->i_out = PidGoal->lim_i_out;
  }

  PidGoal->total_out = PidGoal->p_out + PidGoal->i_out + PidGoal->d_out + PidGoal->f_out; // 计算总和输出

  if (fabs(PidGoal->total_out) > PidGoal->lim_out) // 防止总和过大
  {
    if (PidGoal->total_out < 0)
      PidGoal->total_out = -PidGoal->lim_out;
    else
      PidGoal->total_out = PidGoal->lim_out;
  }

  return PidGoal->total_out;
}
/**
 * @brief 清除 PID 过程变量，防止积分饱和或残留误差影响下一次控制
 * @param PidClear PID 结构体指针
 */
void PID_Clear(pid_t *PidClear)
{
  if (PidClear == NULL)
    return;

  // 清除误差项
  PidClear->set = 0.0f;
  PidClear->err = 0.0f;
  PidClear->err_last = 0.0f;
  PidClear->diff = 0.0f;

  // 清除输出项（特别是关键的 i_out）
  PidClear->p_out = 0.0f;
  PidClear->i_out = 0.0f;
  PidClear->d_out = 0.0f;
  PidClear->f_out = 0.0f;
  PidClear->total_out = 0.0f;
}
