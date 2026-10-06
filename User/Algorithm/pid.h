/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-29 21:45:23
 * @FilePath: \Season-26-Code\User\Algorithm\pid.h
 */
#ifndef __PID_H_
#define __PID_H_

typedef struct // PID控制数据结构体
{
  float p, i, d;                        // 比例，积分，微分
  float set, err, err_last;             // 设置值，误差值，上次的误差值
  float diff;                           // 误差微分
  float p_out, i_out, d_out, total_out; // 比例，积分，微分的输出和总输出
  float lim_i_out, lim_out;             // 输出积分限制，总输出限制
} pid_t;

void PID_Set(pid_t *PidSet, float p_set, float i_set, float d_set, float lim_out_set, float lim_i_outset); // PID设置
float PID_Cal(pid_t *PidGoal, float Now, float Set);                                                       // PID计算

#endif