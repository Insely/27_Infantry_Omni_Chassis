#ifndef __DM_MOTOR_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __DM_MOTOR_CTRL_H__
#include "main.h"
#include "dm_motor_drv.h"

extern uint32_t motor1_data_sent;
extern uint32_t motor2_data_sent;
extern uint32_t motor3_data_sent;
extern uint32_t motor4_data_sent;

typedef union
{
	float f_val;
	uint32_t u_val;
	uint8_t b_val[4];
}float_type_u;

void dm_motor_init(DM_motor_t *motor);

void read_all_motor_data(DM_motor_t *motor);
void receive_motor_data(DM_motor_t *motor, uint8_t *data);


#ifdef __cplusplus
}
#endif
#endif /* __DM_MOTOR_CTRL_H__ */

