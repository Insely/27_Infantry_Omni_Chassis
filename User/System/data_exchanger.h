/**
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       data_exchanger.c/h
  * @brief      数据交换中心，用于各个模块之间的数据交换，不用相互调用头文件，加强各模块之间的解耦
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.11.2        Wang Zihao        1. 完成最初版
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim 
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  */

#ifndef __DATA_EXCHANGER_H
#define __DATA_EXCHANGER_H

#include "struct_typedef.h"

#define NAME_LEN 20

typedef enum __DataExchangeIndex {
    TEST_DATA = 0,
    YAW_ANGLE,
    Data_Exchange_INDEX_NUM
} DataExchangeIndex_e;

typedef enum __Data_Type {
    DE_INT8 = 0,
    DE_UINT8,
    DE_INT16,
    DE_UINT16,
    DE_INT32,
    DE_UINT32,
    DE_FLOAT,
    Data_Type_NUM
} DataType_e;

typedef enum DataPublishStatus {
    PUBLISH_FAIL = 0,
    PUBLISH_OK,
    PUBLISH_ALREADY_EXIST,
    PUBLISH_ALREADY_FULL
} DataPublishStatus_e;

typedef struct
{
    void * data_address;
    uint32_t data_size;
    char data_name[NAME_LEN];
} Data_t;

typedef enum DataSubscribeStatus { SUBSCRIBE_FAIL = 0, SUBSCRIBE_OK } DataSubscribeStatus_e;

extern uint8_t Publish(void * address, char * name);
extern const void * Subscribe(char * name);

#endif  // __DATA_EXCHANGE_H
