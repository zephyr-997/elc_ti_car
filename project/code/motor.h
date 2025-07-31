#ifndef __MOTOR_H___
#define __MOTOR_H___

#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"
#include "zf_common_typedef.h"

// #ifndef  PWM_DUTY_MAX
// #define PWM_DUTY_MAX        ( 10000 )
// #endif

#define  MOTOR_PWM_FREQ   17000   //PWM输出频率
#define  MOTOR_PWM_LIMIT  9000    //PWM输出限幅90%

void motor_init(void);
void set_motor_pwm(int32_t left_duty, int32_t right_duty);

#endif