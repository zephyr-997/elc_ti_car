/*********************************************************************************************************************
* MSPM0G3507 编码器驱动头文件
*
* 支持13线霍尔编码器 + 1:20减速比
* 四倍频解码，GPIO中断方式
*
* 引脚分配：
* E1A -> A15, E1B -> A16 (左编码器)
* E2A -> A17, E2B -> A22 (右编码器)
********************************************************************************************************************/

#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "zf_common_headfile.h"
#include <stdarg.h>
#include <stdio.h>
#include "pid.h"

// 编码器参数配置
#define ENCODER_PPR             13      // 编码器线数
#define GEAR_RATIO              20      // 减速比 1:20
#define EFFECTIVE_PPR           (ENCODER_PPR * GEAR_RATIO)  // 有效分辨率: 260
#define SAMPLE_PERIOD_MS        10     // 采样周期(ms)

// 递推均值滤波配置
#define FILTER_ENABLE           1       // 1: 启用滤波, 0: 禁用滤波
#define FILTER_WINDOW_SIZE      5       // 滤波窗口大小(建议3-10)

// 编码器引脚定义
#define ENCODER1_A_PIN          A15     // 左编码器A相
#define ENCODER1_B_PIN          A16     // 左编码器B相
#define ENCODER2_A_PIN          A17     // 右编码器A相
#define ENCODER2_B_PIN          A22     // 右编码器B相

// 递推均值滤波结构体
typedef struct {
    float buffer[FILTER_WINDOW_SIZE];   // 滤波缓冲区
    uint8_t index;                      // 当前索引
    float sum;                          // 当前总和
    bool is_full;                       // 缓冲区是否已满
} moving_average_filter_t;

// 编码器结构体
typedef struct {
    volatile int32_t count;             // 脉冲计数
    volatile uint8_t last_state;        // 上次状态
    volatile float speed_rpm;           // 转速(RPM) - 原始值
    volatile float speed_rps;           // 转速(RPS) - 原始值
    volatile float filtered_rpm;        // 转速(RPM) - 滤波后
    volatile float filtered_rps;        // 转速(RPS) - 滤波后
    volatile uint32 last_time;          // 上次时间戳
    moving_average_filter_t rpm_filter; // RPM滤波器
    moving_average_filter_t rps_filter; // RPS滤波器
    volatile float target_rpm;          // 目标转速(RPM)
    volatile float pid_output;          // 速度环PID输出值
    volatile int16_t pwm_output;        // 最终PWM输出值
} encoder_data_t;

// 编码器索引
typedef enum {
    ENCODER_LEFT = 0,   // 左编码器
    ENCODER_RIGHT = 1   // 右编码器
} encoder_index_t;

// 电机索引（与编码器对应）
typedef enum {
    MOTOR_LEFT = 0,     // 左电机
    MOTOR_RIGHT = 1     // 右电机
} motor_index_t;

// PWM控制参数
#define MOTOR_PWM_MAX       1000    // 最大PWM值
#define MOTOR_PWM_MIN      -1000    // 最小PWM值

extern PID_t left_speed_pid;
extern PID_t right_speed_pid;


// 函数声明
void encoder_init(void);
void encoder_start(void);
void encoder_stop(void);
int32_t encoder_get_count(encoder_index_t encoder);
float encoder_get_speed_rpm(encoder_index_t encoder);
float encoder_get_speed_rps(encoder_index_t encoder);
float encoder_get_filtered_rpm(encoder_index_t encoder);    // 获取滤波后RPM
float encoder_get_filtered_rps(encoder_index_t encoder);    // 获取滤波后RPS
void encoder_clear_count(encoder_index_t encoder);
void encoder_printf(const char *format, ...);

// PID控制相关函数
void encoder_set_target_rpm(encoder_index_t encoder, float target_rpm);    // 设置目标转速
float encoder_get_target_rpm(encoder_index_t encoder);                     // 获取目标转速
float encoder_get_pid_output(encoder_index_t encoder);                     // 获取速度环PID输出
int16_t encoder_get_pwm_output(encoder_index_t encoder);                   // 获取PWM输出值
void encoder_pid_enable(bool enable);                                      // 启用/禁用PID控制
void encoder_pid_reset(encoder_index_t encoder);                           // 重置PID控制器
void encoder_set_pid_mode(bool use_incremental);                           // 设置PID模式（位置式/增量式）



// 转向环控制相关（预留）
extern volatile float turn_pid_output;   
                                  // 转向环PID输出
void encoder_set_turn_output(float turn_output);                           // 设置转向环输出
float encoder_get_turn_output(void);                                       // 获取转向环输出

// 电机PWM控制接口（在motor.c中实现）
void motor_set_pwm(motor_index_t motor, int16_t pwm_value);                // 设置电机PWM

// 中断处理函数
void encoder_gpio_handler(uint32 event, void *ptr);
void encoder_timer_handler(uint32 event, void *ptr);

#endif