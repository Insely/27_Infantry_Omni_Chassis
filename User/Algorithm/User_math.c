#include "User_math.h"

//快速开方
fp32 Inv_Sqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

//绝对限制
void Abs_Limit(fp32 * num, fp32 Limit)
{
    if (*num > Limit) {
        *num = Limit;
    } else if (*num < -Limit) {
        *num = -Limit;
    }
}

//判断符号位
fp32 Sign(fp32 value)
{
    if (value >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue) {
        Value = 0.0f;
    }
    return Value;
}

//int16死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue) {
        Value = 0;
    }
    return Value;
}

//浮点限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//int16限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//浮点循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue) {
        return Input;
    }

    if (Input > maxValue) {
        fp32 len = maxValue - minValue;
        while (Input > maxValue) {
            Input -= len;
        }
    } else if (Input < minValue) {
        fp32 len = maxValue - minValue;
        while (Input < minValue) {
            Input += len;
        }
    }
    return Input;
}

// 将任意角度归一化到 [-PI, PI]
float normalize_angle(float angle) {
    const float TWO_PI = 2.0 * PI;
    angle = fmod(angle, TWO_PI);      // 约简到 [-2π, 2π)
    if (angle > PI)
        angle -= TWO_PI;
    else if (angle < -PI)
        angle += TWO_PI;
    return angle;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// 将float按字节拆分
void float_to_bytes(float f, uint8_t *bytes) 
	{
    uint32_t *p = (uint32_t *)&f;  // 将float指针强制转换为uint32_t指针
    uint32_t temp = *p;            // 获取float的二进制表示
    bytes[0] = (temp >> 0) & 0xFF; // 提取最低字节
    bytes[1] = (temp >> 8) & 0xFF; // 提取第二个字节
    bytes[2] = (temp >> 16) & 0xFF; // 提取第三个字节
    bytes[3] = (temp >> 24) & 0xFF; // 提取最高字节
  }

// 将字节数组转换为float
float bytes_to_float(uint8_t *bytes) 
	 {
	 uint32_t temp = 0;
	 temp |= (bytes[0] << 0);  // 最低字节
	 temp |= (bytes[1] << 8);  // 第二个字节
	 temp |= (bytes[2] << 16); // 第三个字节
	 temp |= (bytes[3] << 24); // 最高字节
	 return *(float *)&temp;   // 将uint32_t指针强制转换为float指针
	 }

// 将uint16_t按字节拆分
void uint16_to_bytes(uint16_t value, uint8_t *bytes) 
{
    bytes[0] = (value >> 0) & 0xFF; // 低位字节
    bytes[1] = (value >> 8) & 0xFF; // 高位字节
}

// 将字节数组转换为uint16_t
uint16_t bytes_to_uint16(uint8_t *bytes) 
{
    uint16_t temp = 0;
    temp |= ((uint16_t)bytes[0] << 0);
    temp |= ((uint16_t)bytes[1] << 8);
    return temp;
}
// 将uint8_t按字节拆分（其实就是赋值）
void uint8_to_bytes(uint8_t value, uint8_t *bytes) 
{
    bytes[0] = value;
}

// 将字节数组转换为uint8_t
uint8_t bytes_to_uint8(uint8_t *bytes) 
{
    return bytes[0];
}

void int_to_bytes(int value, uint8_t *bytes) 
{
    bytes[0] = (value >> 0) & 0xFF; // 低位字节
    bytes[1] = (value >> 8) & 0xFF; // 高位字节
    bytes[2] = (value >> 16) & 0xFF; // 第三字节
    bytes[3] = (value >> 24) & 0xFF; // 第四字节
}

int bytes_to_int(uint8_t *bytes) 
{
    int temp = 0;
    temp |= ((int)bytes[0] << 0);
    temp |= ((int)bytes[1] << 8);
    temp |= ((int)bytes[2] << 16);
    temp |= ((int)bytes[3] << 24);
    return temp;
}