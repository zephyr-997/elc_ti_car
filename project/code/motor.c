#include "motor.h"

void motor_init(void)
{
	/* 左轮初始化 */
    pwm_init(PWM_TIM_A1_CH0_B4, 17000, 0);	
	gpio_init(A13, GPO, 0, GPO_PUSH_PULL);
	gpio_init(A12, GPO, 0, GPO_PUSH_PULL);
	/* 右轮初始化 */
    pwm_init(PWM_TIM_G8_CH1_B11, 17000, 0);	
	gpio_init(B0, GPO, 0, GPO_PUSH_PULL);
	gpio_init(B16, GPO, 0, GPO_PUSH_PULL);
}

void set_motor_pwm(int32_t left_duty, int32_t right_duty)
{
	/* 左轮PWM限幅并输出 */
	if(left_duty >= 0)
	{
		if (left_duty > PWM_DUTY_MAX)
		{
			left_duty = PWM_DUTY_MAX;
		}
	
        pwm_set_duty(PWM_TIM_A1_CH0_B4, left_duty);
		gpio_set_level(A13, 1);
        gpio_set_level(A12, 0);

	}
	else
	{
		if (left_duty < -MOTOR_PWM_LIMIT)
		{
			left_duty = -MOTOR_PWM_LIMIT;
		}
	
        pwm_set_duty(PWM_TIM_A1_CH0_B4, left_duty);
		gpio_set_level(A13, 0);
        gpio_set_level(A12, 1);
	}
	
	/* 右轮PWM限幅并输出 */
	if (right_duty >= 0)
	{
		if (right_duty > MOTOR_PWM_LIMIT)
		{
			right_duty = MOTOR_PWM_LIMIT;
		}
		
        pwm_set_duty(PWM_TIM_G8_CH1_B11, right_duty);
		gpio_set_level(B0, 1);
        gpio_set_level(B16, 0);
	}
	else	
	{
		if (right_duty < -MOTOR_PWM_LIMIT)
		{
			right_duty = -MOTOR_PWM_LIMIT;
		}
		
        pwm_set_duty(PWM_TIM_G8_CH1_B11, right_duty);
		gpio_set_level(B0, 0);
        gpio_set_level(B16, 1);
	}
}