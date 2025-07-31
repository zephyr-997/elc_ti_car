#include "motor.h"

static bool motor_initialized = false;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机初始化
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    if(motor_initialized) return;

	/* 左轮初始化 */
    pwm_init(MOTOR_LEFT_PWM_PIN, MOTOR_PWM_FREQ, 0);
	gpio_init(MOTOR_LEFT_DIR1_PIN, GPO, 0, GPO_PUSH_PULL);
	gpio_init(MOTOR_LEFT_DIR2_PIN, GPO, 0, GPO_PUSH_PULL);

	/* 右轮初始化 */
    pwm_init(MOTOR_RIGHT_PWM_PIN, MOTOR_PWM_FREQ, 0);
	gpio_init(MOTOR_RIGHT_DIR1_PIN, GPO, 0, GPO_PUSH_PULL);
	gpio_init(MOTOR_RIGHT_DIR2_PIN, GPO, 0, GPO_PUSH_PULL);

	motor_initialized = true;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     原有的双电机控制函数（保持兼容性）
// 参数说明     left_duty       左轮PWM占空比
// 参数说明     right_duty      右轮PWM占空比
//-------------------------------------------------------------------------------------------------------------------
void set_motor_pwm(int32_t left_duty, int32_t right_duty)
{
	/* 左轮PWM限幅并输出 */
	if(left_duty >= 0)
	{
		if (left_duty > MOTOR_PWM_LIMIT)
		{
			left_duty = MOTOR_PWM_LIMIT;
		}

        pwm_set_duty(MOTOR_LEFT_PWM_PIN, left_duty);
		gpio_set_level(MOTOR_LEFT_DIR1_PIN, 1);
        gpio_set_level(MOTOR_LEFT_DIR2_PIN, 0);

	}
	else
	{
		if (left_duty < -MOTOR_PWM_LIMIT)
		{
			left_duty = -MOTOR_PWM_LIMIT;
		}

        pwm_set_duty(MOTOR_LEFT_PWM_PIN, -left_duty);  // 使用绝对值
		gpio_set_level(MOTOR_LEFT_DIR1_PIN, 0);
        gpio_set_level(MOTOR_LEFT_DIR2_PIN, 1);
	}

	/* 右轮PWM限幅并输出 */
	if (right_duty >= 0)
	{
		if (right_duty > MOTOR_PWM_LIMIT)
		{
			right_duty = MOTOR_PWM_LIMIT;
		}

        pwm_set_duty(MOTOR_RIGHT_PWM_PIN, right_duty);
		gpio_set_level(MOTOR_RIGHT_DIR1_PIN, 1);
        gpio_set_level(MOTOR_RIGHT_DIR2_PIN, 0);
	}
	else
	{
		if (right_duty < -MOTOR_PWM_LIMIT)
		{
			right_duty = -MOTOR_PWM_LIMIT;
		}

        pwm_set_duty(MOTOR_RIGHT_PWM_PIN, -right_duty);  // 使用绝对值
		gpio_set_level(MOTOR_RIGHT_DIR1_PIN, 0);
        gpio_set_level(MOTOR_RIGHT_DIR2_PIN, 1);
	}
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单电机PWM控制函数（encoder系统调用）
// 参数说明     motor           电机索引 (MOTOR_LEFT 或 MOTOR_RIGHT)
// 参数说明     pwm_value       PWM值 (-1000 ~ +1000)
//-------------------------------------------------------------------------------------------------------------------
void motor_set_pwm(motor_index_t motor, int16_t pwm_value)
{
    // 确保电机已初始化
    if(!motor_initialized) {
        motor_init();
    }

    // PWM值限幅到±1000
    if(pwm_value > 1000) pwm_value = 1000;
    if(pwm_value < -1000) pwm_value = -1000;

    // 将±1000映射到实际PWM范围
    int32_t actual_pwm = (int32_t)pwm_value * MOTOR_PWM_LIMIT / 1000;

    if(motor == MOTOR_LEFT) {
        // 左电机控制
        if(actual_pwm >= 0) {
            // 正转
            pwm_set_duty(MOTOR_LEFT_PWM_PIN, actual_pwm);
            gpio_set_level(MOTOR_LEFT_DIR1_PIN, 1);
            gpio_set_level(MOTOR_LEFT_DIR2_PIN, 0);
        } else {
            // 反转
            pwm_set_duty(MOTOR_LEFT_PWM_PIN, -actual_pwm);
            gpio_set_level(MOTOR_LEFT_DIR1_PIN, 0);
            gpio_set_level(MOTOR_LEFT_DIR2_PIN, 1);
        }
    } else if(motor == MOTOR_RIGHT) {
        // 右电机控制
        if(actual_pwm >= 0) {
            // 正转
            pwm_set_duty(MOTOR_RIGHT_PWM_PIN, actual_pwm);
            gpio_set_level(MOTOR_RIGHT_DIR1_PIN, 1);
            gpio_set_level(MOTOR_RIGHT_DIR2_PIN, 0);
        } else {
            // 反转
            pwm_set_duty(MOTOR_RIGHT_PWM_PIN, -actual_pwm);
            gpio_set_level(MOTOR_RIGHT_DIR1_PIN, 0);
            gpio_set_level(MOTOR_RIGHT_DIR2_PIN, 1);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止所有电机
//-------------------------------------------------------------------------------------------------------------------
void motor_stop_all(void)
{
    motor_set_pwm(MOTOR_LEFT, 0);
    motor_set_pwm(MOTOR_RIGHT, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     紧急停止所有电机
//-------------------------------------------------------------------------------------------------------------------
void motor_emergency_stop(void)
{
    // 立即停止PWM输出
    pwm_set_duty(MOTOR_LEFT_PWM_PIN, 0);
    pwm_set_duty(MOTOR_RIGHT_PWM_PIN, 0);

    // 设置所有方向控制引脚为低电平
    gpio_set_level(MOTOR_LEFT_DIR1_PIN, 0);
    gpio_set_level(MOTOR_LEFT_DIR2_PIN, 0);
    gpio_set_level(MOTOR_RIGHT_DIR1_PIN, 0);
    gpio_set_level(MOTOR_RIGHT_DIR2_PIN, 0);
}