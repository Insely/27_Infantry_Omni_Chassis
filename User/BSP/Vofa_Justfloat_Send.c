#include "USB_VirCom.h"
#include <string.h>

void Vofa_SendFloat(float *data, uint8_t num)
{
    
    static uint8_t tx_buf[4 * 8 + 4];  // 最多8通道，按需改大
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

    if (num > 8) num = 8;

    memcpy(tx_buf, data, num * sizeof(float));
    memcpy(tx_buf + num * sizeof(float), tail, 4);

    Vircom_Send(tx_buf, num * sizeof(float) + 4);

}