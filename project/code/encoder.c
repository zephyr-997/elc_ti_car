/*********************************************************************************************************************
* MSPM0G3507 编码器驱动实现文件
*
* 支持13线霍尔编码器 + 1:20减速比
* 四倍频解码，GPIO中断方式
*
* 功能特点：
* 1. 双编码器支持（左右轮）
* 2. 四倍频解码，高精度
* 3. 实时速度计算
* 4. GPIO中断触发，响应快速
********************************************************************************************************************/

#include "encoder.h"

// 编码器数据
static encoder_data_t encoder_data[2];
static volatile bool encoder_initialized = false;
static volatile bool timer_enabled = false;
static volatile bool pid_enabled = false;
static volatile bool use_incremental_pid = true;  // 默认使用增量式PID

// PID控制器 - 左右轮速度控制
PID_t left_speed_pid;
PID_t right_speed_pid;

// 转向环输出变量（预留）
volatile float turn_pid_output = 0;

// 状态转换表 - 四倍频解码
// 格雷码序列: 00 -> 01 -> 11 -> 10 -> 00 (正转)
// 格雷码序列: 00 -> 10 -> 11 -> 01 -> 00 (反转)
static const int8_t state_table[16] = {
    0,  1, -1,  0,   // 00: 00->00(0), 00->01(+1), 00->10(-1), 00->11(0)
   -1,  0,  0,  1,   // 01: 01->00(-1), 01->01(0), 01->10(0), 01->11(+1)
    1,  0,  0, -1,   // 10: 10->00(+1), 10->01(0), 10->10(0), 10->11(-1)
    0, -1,  1,  0    // 11: 11->00(0), 11->01(-1), 11->10(+1), 11->11(0)
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取编码器引脚状态
// 参数说明     encoder         编码器索引
// 返回参数     uint8_t         当前状态 (bit1:B相, bit0:A相)
//-------------------------------------------------------------------------------------------------------------------
static uint8_t encoder_read_pins(encoder_index_t encoder)
{
    uint8_t state = 0;

    if(encoder == ENCODER_LEFT) {
        if(gpio_get_level(ENCODER1_A_PIN)) state |= 0x01;
        if(gpio_get_level(ENCODER1_B_PIN)) state |= 0x02;
    } else {
        if(gpio_get_level(ENCODER2_A_PIN)) state |= 0x01;
        if(gpio_get_level(ENCODER2_B_PIN)) state |= 0x02;
    }

    return state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器状态处理
// 参数说明     encoder         编码器索引
// 参数说明     current_state   当前状态
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     递推均值滤波器初始化
// 参数说明     filter          滤波器指针
//-------------------------------------------------------------------------------------------------------------------
static void moving_average_init(moving_average_filter_t *filter)
{
    int i = 0;
    for(i = 0; i < FILTER_WINDOW_SIZE; i++) {
        filter->buffer[i] = 0.0f;
    }
    filter->index = 0;
    filter->sum = 0.0f;
    filter->is_full = false;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     递推均值滤波器更新
// 参数说明     filter          滤波器指针
// 参数说明     new_value       新的输入值
// 返回参数     float           滤波后的值
//-------------------------------------------------------------------------------------------------------------------
static float moving_average_update(moving_average_filter_t *filter, float new_value)
{
#if FILTER_ENABLE
    // 移除旧值（如果缓冲区已满）
    if(filter->is_full) {
        filter->sum -= filter->buffer[filter->index];
    }

    // 添加新值
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;

    // 更新索引
    filter->index = (filter->index + 1) % FILTER_WINDOW_SIZE;

    // 检查缓冲区是否已满
    if(!filter->is_full && filter->index == 0) {
        filter->is_full = true;
    }

    // 计算平均值
    uint8_t count = filter->is_full ? FILTER_WINDOW_SIZE : filter->index;
    return (count > 0) ? (filter->sum / count) : 0.0f;
#else
    // 滤波禁用时直接返回原值
    (void)filter;
    return new_value;
#endif
}

static void encoder_process_state(encoder_index_t encoder, uint8_t current_state)
{
    uint8_t last_state = encoder_data[encoder].last_state;
    uint8_t state_index = (last_state << 2) | current_state;

    // 使用状态转换表进行解码
    int8_t direction = state_table[state_index];
    encoder_data[encoder].count += direction;
    encoder_data[encoder].last_state = current_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器GPIO中断处理
// 参数说明     event           中断事件
// 参数说明     ptr             回调参数指针
//-------------------------------------------------------------------------------------------------------------------
void encoder_gpio_handler(uint32 event, void *ptr)
{
    (void)event;  // 未使用的参数
    (void)ptr;    // 未使用的参数

    // 处理左编码器 - 检查A15和A16引脚
    uint8_t current_state_left = encoder_read_pins(ENCODER_LEFT);
    encoder_process_state(ENCODER_LEFT, current_state_left);

    // 处理右编码器 - 检查A17和A22引脚
    uint8_t current_state_right = encoder_read_pins(ENCODER_RIGHT);
    encoder_process_state(ENCODER_RIGHT, current_state_right);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器定时器中断处理 - 计算速度并执行PID控制
// 参数说明     event           中断事件
// 参数说明     ptr             回调参数指针
//-------------------------------------------------------------------------------------------------------------------
void encoder_timer_handler(uint32 event, void *ptr)
{
    float final_output = 0;
    int i = 0;
    (void)event;  // 未使用的参数
    (void)ptr;    // 未使用的参数

    if(!timer_enabled) return;

    for(i = 0; i < 2; i++) {
        // 计算原始转速(RPM)
        // 公式: RPM = (脉冲数 / (有效PPR * 4)) * (60000 / 采样周期ms)
        encoder_data[i].speed_rpm = (float)encoder_data[i].count / (EFFECTIVE_PPR * 4.0f) * (60000.0f / SAMPLE_PERIOD_MS);

        // 计算原始转速(RPS)
        encoder_data[i].speed_rps = encoder_data[i].speed_rpm / 60.0f;

        // 应用递推均值滤波
        encoder_data[i].filtered_rpm = moving_average_update(&encoder_data[i].rpm_filter, encoder_data[i].speed_rpm);
        encoder_data[i].filtered_rps = moving_average_update(&encoder_data[i].rps_filter, encoder_data[i].speed_rps);

        // PID控制逻辑
        if(pid_enabled) {
            if(i == ENCODER_LEFT) {
                // 左轮PID控制
                if(use_incremental_pid) {
                    encoder_data[i].pid_output = pid_increment(&left_speed_pid,
                                                             encoder_data[i].filtered_rpm,
                                                             encoder_data[i].target_rpm);
                } else {
                    encoder_data[i].pid_output = pid_poisitional(&left_speed_pid,
                                                               encoder_data[i].filtered_rpm,
                                                               encoder_data[i].target_rpm);
                }
            } else {
                // 右轮PID控制
                if(use_incremental_pid) {
                    encoder_data[i].pid_output = pid_increment(&right_speed_pid,
                                                             encoder_data[i].filtered_rpm,
                                                             encoder_data[i].target_rpm);
                } else {
                    encoder_data[i].pid_output = pid_poisitional(&right_speed_pid,
                                                               encoder_data[i].filtered_rpm,
                                                               encoder_data[i].target_rpm);
                }
            }
        } else {
            encoder_data[i].pid_output = 0.0f;
        }

        // PWM输出计算和控制
        if(pid_enabled) {
            final_output = encoder_data[i].pid_output;

            // 如果是差速控制，需要叠加转向环输出
            if(i == ENCODER_LEFT) {
                final_output -= turn_pid_output;  // 左轮减去转向输出
            } else {
                final_output += turn_pid_output;  // 右轮加上转向输出
            }

            // PWM限幅
            if(final_output > MOTOR_PWM_MAX) {
                final_output = MOTOR_PWM_MAX;
            } else if(final_output < MOTOR_PWM_MIN) {
                final_output = MOTOR_PWM_MIN;
            }

            encoder_data[i].pwm_output = (int16_t)final_output;

            // 输出PWM到电机
            motor_set_pwm((motor_index_t)i, encoder_data[i].pwm_output);
        } else {
            // PID禁用时，停止电机
            encoder_data[i].pwm_output = 0;
            motor_set_pwm((motor_index_t)i, 0);
        }

        // 重置计数器
        encoder_data[i].count = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器初始化
//-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
    if(encoder_initialized) return;

    // 初始化编码器数据
    for(int i = 0; i < 2; i++) {
        encoder_data[i].count = 0;
        encoder_data[i].last_state = 0;
        encoder_data[i].speed_rpm = 0.0f;
        encoder_data[i].speed_rps = 0.0f;
        encoder_data[i].filtered_rpm = 0.0f;
        encoder_data[i].filtered_rps = 0.0f;
        encoder_data[i].last_time = 0;
        encoder_data[i].target_rpm = 0.0f;
        encoder_data[i].pid_output = 0.0f;
        encoder_data[i].pwm_output = 0;

        // 初始化滤波器
        moving_average_init(&encoder_data[i].rpm_filter);
        moving_average_init(&encoder_data[i].rps_filter);
    }

    // 初始化PID控制器
    // 左轮速度PID - 使用适中的参数
    pid_init(&left_speed_pid, 10.0f, 0.0f, 0.0f, 100.0f, 1000.0f);

    // 右轮速度PID - 使用适中的参数
    pid_init(&right_speed_pid, 0.0f, 0.0f, 0.0f, 100.0f, 1000.0f);

    // 配置编码器引脚为输入，启用上拉
    gpio_init(ENCODER1_A_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(ENCODER1_B_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(ENCODER2_A_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(ENCODER2_B_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    // 读取初始状态
    encoder_data[ENCODER_LEFT].last_state = encoder_read_pins(ENCODER_LEFT);
    encoder_data[ENCODER_RIGHT].last_state = encoder_read_pins(ENCODER_RIGHT);

    // 配置GPIO中断 - 双边沿触发
    exti_init(ENCODER1_A_PIN, EXTI_TRIGGER_BOTH, encoder_gpio_handler, NULL);
    exti_init(ENCODER1_B_PIN, EXTI_TRIGGER_BOTH, encoder_gpio_handler, NULL);
    exti_init(ENCODER2_A_PIN, EXTI_TRIGGER_BOTH, encoder_gpio_handler, NULL);
    exti_init(ENCODER2_B_PIN, EXTI_TRIGGER_BOTH, encoder_gpio_handler, NULL);

    // 配置定时器用于周期性计算速度
    pit_ms_init(PIT_TIM_G0, SAMPLE_PERIOD_MS, encoder_timer_handler, NULL);

    encoder_initialized = true;

    encoder_printf("编码器初始化完成\r\n");
    encoder_printf("编码器线数: %d, 减速比: 1:%d\r\n", ENCODER_PPR, GEAR_RATIO);
    encoder_printf("有效分辨率: %d 脉冲/转\r\n", EFFECTIVE_PPR * 4);
    encoder_printf("采样周期: %d ms\r\n", SAMPLE_PERIOD_MS);
#if FILTER_ENABLE
    encoder_printf("递推均值滤波: 启用, 窗口大小: %d\r\n", FILTER_WINDOW_SIZE);
#else
    encoder_printf("递推均值滤波: 禁用\r\n");
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动编码器
//-------------------------------------------------------------------------------------------------------------------
void encoder_start(void)
{
    if(!encoder_initialized) return;

    timer_enabled = true;
    pit_enable(PIT_TIM_G0);

    encoder_printf("编码器已启动\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止编码器
//-------------------------------------------------------------------------------------------------------------------
void encoder_stop(void)
{
    if(!encoder_initialized) return;

    timer_enabled = false;
    pit_disable(PIT_TIM_G0);

    encoder_printf("编码器已停止\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器计数值
// 参数说明     encoder         编码器索引
// 返回参数     int32_t         当前计数值
//-------------------------------------------------------------------------------------------------------------------
int32_t encoder_get_count(encoder_index_t encoder)
{
    if(encoder >= 2) return 0;
    return encoder_data[encoder].count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器转速(RPM)
// 参数说明     encoder         编码器索引
// 返回参数     float           转速(RPM)
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_speed_rpm(encoder_index_t encoder)
{
    if(encoder >= 2) return 0.0f;
    return encoder_data[encoder].speed_rpm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器转速(RPS)
// 参数说明     encoder         编码器索引
// 返回参数     float           转速(RPS)
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_speed_rps(encoder_index_t encoder)
{
    if(encoder >= 2) return 0.0f;
    return encoder_data[encoder].speed_rps;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器滤波后转速(RPM)
// 参数说明     encoder         编码器索引
// 返回参数     float           滤波后转速(RPM)
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_filtered_rpm(encoder_index_t encoder)
{
    if(encoder >= 2) return 0.0f;
    return encoder_data[encoder].filtered_rpm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器滤波后转速(RPS)
// 参数说明     encoder         编码器索引
// 返回参数     float           滤波后转速(RPS)
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_filtered_rps(encoder_index_t encoder)
{
    if(encoder >= 2) return 0.0f;
    return encoder_data[encoder].filtered_rps;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除编码器计数
// 参数说明     encoder         编码器索引
//-------------------------------------------------------------------------------------------------------------------
void encoder_clear_count(encoder_index_t encoder)
{
    if(encoder >= 2) return;
    encoder_data[encoder].count = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置目标转速
// 参数说明     encoder         编码器索引
// 参数说明     target_rpm      目标转速(RPM)
//-------------------------------------------------------------------------------------------------------------------
void encoder_set_target_rpm(encoder_index_t encoder, float target_rpm)
{
    if(encoder >= 2) return;
    encoder_data[encoder].target_rpm = target_rpm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取目标转速
// 参数说明     encoder         编码器索引
// 返回参数     float           目标转速(RPM)
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_target_rpm(encoder_index_t encoder)
{
    if(encoder >= 2) return 0.0f;
    return encoder_data[encoder].target_rpm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取PID输出值
// 参数说明     encoder         编码器索引
// 返回参数     float           PID输出值
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_pid_output(encoder_index_t encoder)
{
    if(encoder >= 2) return 0.0f;
    return encoder_data[encoder].pid_output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启用/禁用PID控制
// 参数说明     enable          true:启用, false:禁用
//-------------------------------------------------------------------------------------------------------------------
void encoder_pid_enable(bool enable)
{
    pid_enabled = enable;
    if(enable) {
        encoder_printf("PID控制已启用\r\n");
    } else {
        encoder_printf("PID控制已禁用\r\n");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置PID控制器
// 参数说明     encoder         编码器索引
//-------------------------------------------------------------------------------------------------------------------
void encoder_pid_reset(encoder_index_t encoder)
{
    if(encoder >= 2) return;

    if(encoder == ENCODER_LEFT) {
        pid_clean(&left_speed_pid);
    } else {
        pid_clean(&right_speed_pid);
    }

    encoder_data[encoder].pid_output = 0.0f;
    encoder_printf("编码器%d PID控制器已重置\r\n", encoder);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID控制模式
// 参数说明     use_incremental true:增量式PID, false:位置式PID
//-------------------------------------------------------------------------------------------------------------------
void encoder_set_pid_mode(bool use_incremental)
{
    use_incremental_pid = use_incremental;

    // 切换模式时重置PID控制器
    pid_clean(&left_speed_pid);
    pid_clean(&right_speed_pid);

    encoder_printf("PID控制模式设置为: %s\r\n",
                   use_incremental ? "增量式" : "位置式");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取PWM输出值
// 参数说明     encoder         编码器索引
// 返回参数     int16_t         PWM输出值
//-------------------------------------------------------------------------------------------------------------------
int16_t encoder_get_pwm_output(encoder_index_t encoder)
{
    if(encoder >= 2) return 0;
    return encoder_data[encoder].pwm_output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置转向环输出
// 参数说明     turn_output     转向环PID输出值
//-------------------------------------------------------------------------------------------------------------------
void encoder_set_turn_output(float turn_output)
{
    turn_pid_output = turn_output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取转向环输出
// 返回参数     float           转向环PID输出值
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_turn_output(void)
{
    return turn_pid_output;
}

// motor_set_pwm函数已在motor.c中实现

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器调试输出函数
// 参数说明     format          格式化字符串
// 参数说明     ...             可变参数
//-------------------------------------------------------------------------------------------------------------------
void encoder_printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // 通过UART0输出调试信息
    for(int i = 0; i < len; i++) {
        uart_write_byte(UART_0, buffer[i]);
    }
}