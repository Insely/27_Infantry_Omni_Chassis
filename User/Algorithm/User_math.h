/*
 * @Date: 2025-09-16 22:22:17
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-09-17 20:11:59
 * @FilePath: \Season-26-Code\User\Algorithm\User_math.h
 */
#ifndef __USER_MATH__
#define __USER_MATH__

#include "math.h"
#include "struct_typedef.h"

//常用数据
#define PI	3.14159265358979f
#define SQRT1_2 0.70710678118f  // 1/sqrt(2)
#define SQRT2   1.41421356237f  // sqrt(2)


//单位转换
#define RAD_S_TO_RPM (30.0f/PI)
#define RPM_TO_RAD_S (PI / 30.0f) //Rpm转化为角度度
#define DEG_TO_RAD (PI / 180.0f) //角度转化为弧度
#define RAD_TO_DEG (180.0f / PI) //弧度转化为角度

//声明一些函数
extern fp32 Inv_Sqrt(fp32 num); //快速开方
extern void Abs_Limit(fp32 * num, fp32 Limit); //绝对限制
extern fp32 Sign(fp32 value); //判断符号
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue); //浮点死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue); //int16死区
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue); //浮点限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue); //int16限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue); //浮点循环限幅函数
extern float normalize_angle(float angle);

//数据类型转换
extern int float_to_uint(float x_float, float x_min, float x_max, int bits) ;
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
void float_to_bytes(float f, uint8_t *bytes);
float bytes_to_float(uint8_t *bytes);
void uint16_to_bytes(uint16_t value, uint8_t *bytes);
uint16_t bytes_to_uint16(uint8_t *bytes);
void uint8_to_bytes(uint8_t value, uint8_t *bytes);
uint8_t bytes_to_uint8(uint8_t *bytes);
void int_to_bytes(int value, uint8_t *bytes);
int bytes_to_int(uint8_t *bytes);
#endif // !__USER_MATH__