#include "BoardLink.h"
#include <string.h>
FDCAN_HandleTypeDef hfdcan1={1},hfdcan2={2},hfdcan3={3};
UART_HandleTypeDef huart1={1},huart2={2},huart3={3};
static uint32_t tick;
static int count,fail_tx;
static uint8_t packets[32][128];
static int sizes[32],ids[32];
uint32_t HAL_GetTick(void) { return tick; }
void Test_Time(uint32_t n) { tick=n; }
void Test_Fail(int f) { fail_tx=f; }
FDCAN_HandleTypeDef *Get_CanHandle(uint8_t b) { return b==0 ? &hfdcan1 : b==1 ? &hfdcan2 : &hfdcan3; }
uint8_t Fdcanx_SendData(FDCAN_HandleTypeDef *c,uint16_t id,uint8_t *d,uint32_t n)
{
    (void)c; if (fail_tx || count>=32) return 1;
    memcpy(packets[count],d,n); sizes[count]=n; ids[count++]=id; return 0;
}
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *d,uint16_t n)
{
    (void)u; return Fdcanx_SendData(0,0,d,n);
}
void Test_Clear(void) { count=0; }
int Test_Count(void) { return count; }
int Test_Packet(int i,uint8_t *d) { memcpy(d,packets[i],sizes[i]); return sizes[i]; }
int Test_Id(int i) { return ids[i]; }
void Test_Complete(void) { BoardLink_UartTxCpltCallback(BOARD_LINK_TRANSPORT==3 ? &huart1 : &huart2); }
int Test_Can(int bus,int id,uint8_t *d) { return BoardLink_RxDispatch(Get_CanHandle(bus),id,d); }
int Test_Serial(int port,uint8_t *d,int n) { return BoardLink_UartRxDispatch(port==1 ? &huart1 : &huart2,d,n); }
void Test_Set(void)
{
    BoardLink.vx=1.25f; BoardLink.vy=-2.5f; BoardLink.w=0.75f; BoardLink.yaw_speed_cmd=-3.2f;
    BoardLink.control_mode=BL_KEY; BoardLink.chassis_mode=BL_SPIN_N;
    BoardLink.trigger_mode=BL_SINGLE; BoardLink.shoot_mode=1; BoardLink.auto_mode=1;
    BoardLink.cap_mode=1; BoardLink.reset=0; BoardLink.ui_sequence=37;
    BoardLink.yaw_angle_cnt=179.2f; BoardLink.yaw_spd=50.0f; BoardLink.body_gyro_z=-1.75f;
    BoardLink.initial_speed=24.5f; BoardLink.barrel_heat=77; BoardLink.heat_limit=240;
    BoardLink.projectile_allowance_17mm=333; BoardLink.launching_frequency=12;
}
float Test_Get(int key)
{
    switch(key) {
    case 0:return BoardLink.vx; case 1:return BoardLink.vy; case 2:return BoardLink.w;
    case 3:return BoardLink.yaw_speed_cmd; case 4:return BoardLink.control_mode;
    case 5:return BoardLink.chassis_mode; case 6:return BoardLink.trigger_mode;
    case 7:return BoardLink.shoot_mode; case 8:return BoardLink.cap_mode;
    case 9:return BoardLink.ui_sequence; case 10:return BoardLink.yaw_angle_cnt;
    case 11:return BoardLink.body_gyro_z; case 12:return BoardLink.initial_speed;
    case 13:return BoardLink.barrel_heat; case 14:return BoardLink.heat_limit;
    case 15:return BoardLink.projectile_allowance_17mm; case 16:return BoardLink.launching_frequency;
    default:return -999;
    }
}
