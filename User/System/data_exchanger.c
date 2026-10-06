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
#include "data_exchanger.h"

#include "clist.h"
#include "stdlib.h"
#include "string.h"

#define DATA_LIST_LEN 10


static List * DATA_LIST = NULL;
static uint8_t USED_LEN = 0;  // 已经使用的数据量


/**
 * @brief          发布数据
 * @param[in]      address 数据地址
 * @param[in]      name 数据名称(最大长度为19字符)
 * @retval         数据发布状态
 */
uint8_t Publish(void * address, char * name)
{
    if (DATA_LIST == NULL) {
        DATA_LIST = ListCreate();
    }

    Node * node = ListGetHead(DATA_LIST);
    // 遍历链表判断数据是否已经存在
    while (node != NULL) {
        if (strcmp(((Data_t *)node->data)->data_name, name) == 0) {
            return PUBLISH_ALREADY_EXIST;
        }
        node = ListGetNodeNext(node);
    }

    // 保存数据
    Data_t * data = (Data_t *)malloc(sizeof(Data_t));
    memcpy(&data->data_address, &address, 4);
    memcpy(data->data_name, name, NAME_LEN);
    USED_LEN++;

    ListPushBack(DATA_LIST, data);

    return PUBLISH_OK;
}

/**
 * @brief          订阅数据
 * @param[in]      name 数据名称
 * @retval         订阅数据的地址
 */
const void * Subscribe(char * name)
{
    if (DATA_LIST == NULL) {
        return NULL;
    }

    // 遍历链表寻找数据
    Node * node = ListGetHead(DATA_LIST);
    while (node != NULL) {
        if (strcmp(((Data_t *)node->data)->data_name, name) == 0) {
            return ((Data_t *)node->data)->data_address;
        }
        node = ListGetNodeNext(node);
    }

    return NULL;
}
