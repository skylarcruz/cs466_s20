#ifndef __MOTOR_H_INCLUDED_
#define __MOTOR_H_INCLUDED_

typedef enum {M_BUSY, M_IDLE} motor_status_t;

void motor_init(void);
void motor_move(uint32_t pos_in_tics);
motor_status_t motor_status(void);

#endif
