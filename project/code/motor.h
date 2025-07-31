#ifndef __MOTOR_H___
#define __MOTOR_H___

#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"
#include "zf_common_typedef.h"

// 包含encoder.h以使用motor_index_t类型定义
#include "encoder.h"

// PWM参数定义
#define  MOTOR_PWM_FREQ   17000   //PWM输出频率
#define  MOTOR_PWM_LIMIT  9000    //PWM输出限幅90%(最大10000)

// 电机引脚定义
#define MOTOR_LEFT_PWM_PIN      PWM_TIM_A1_CH0_B4   // 左电机PWM引脚
#define MOTOR_LEFT_DIR1_PIN     A13                 // 左电机方向控制1
#define MOTOR_LEFT_DIR2_PIN     A12                 // 左电机方向控制2

#define MOTOR_RIGHT_PWM_PIN     PWM_TIM_G8_CH1_B11  // 右电机PWM引脚
#define MOTOR_RIGHT_DIR1_PIN    B0                  // 右电机方向控制1
#define MOTOR_RIGHT_DIR2_PIN    B16                 // 右电机方向控制2

// 函数声明
void motor_init(void);                                              // 电机初始化
void set_motor_pwm(int32_t left_duty, int32_t right_duty);         // 原有的双电机控制函数
void motor_set_pwm(motor_index_t motor, int16_t pwm_value);         // 新的单电机控制函数（encoder系统调用）
void motor_stop_all(void);                                         // 停止所有电机
void motor_emergency_stop(void);                                   // 紧急停止

#endif