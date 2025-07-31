/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3507 开源库的一部分
* 
* MSPM0G3507 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          mian
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "encoder.h"
#include "motor.h"

// 输出模式选择
#define OUTPUT_MODE_VOFA        1   // vofa+纯数据模式
#define OUTPUT_MODE_DEBUG       0   // 调试信息模式

#define CURRENT_OUTPUT_MODE     OUTPUT_MODE_VOFA
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

int main (void)
{
    char uart_buffer[300];
    int32_t left_count = 0;
    int32_t right_count;
    float left_rpm_raw = 0.0f;
    float right_rpm_raw = 0.0f;
    float left_rpm_filtered = 0.0f;
    float right_rpm_filtered = 0.0f;
    float left_target_rpm = 0.0f;
    float right_target_rpm = 0.0f;
    float left_pid_output = 0.0f;
    float right_pid_output = 0.0f;
    int16_t left_pwm_output = 0;
    int16_t right_pwm_output = 0;
    uint32 loop_count = 0;
    float new_target = 0;

    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
	// 此处编写用户代码 例如外设初始化代码等
    uart_init(UART_0, 115200, UART0_TX_A10, UART0_RX_A11);
    gpio_init(B22, GPO, 1, GPO_PUSH_PULL);

    // 初始化电机
    motor_init();

    // 初始化编码器
    encoder_init();
    // 启动编码器
    encoder_start();

    system_delay_ms(500);

    // 启用PID控制
    encoder_pid_enable(true);

    // 设置目标转速 - 示例：左轮100RPM，右轮100RPM
    encoder_set_target_rpm(ENCODER_LEFT, 100.0f);
    encoder_set_target_rpm(ENCODER_RIGHT, 100.0f);

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        gpio_toggle_level(B22);

        // 读取编码器数据
        left_rpm_raw = encoder_get_speed_rpm(ENCODER_LEFT);
        right_rpm_raw = encoder_get_speed_rpm(ENCODER_RIGHT);
        left_rpm_filtered = encoder_get_filtered_rpm(ENCODER_LEFT);
        right_rpm_filtered = encoder_get_filtered_rpm(ENCODER_RIGHT);

        // 读取PID控制数据
        left_target_rpm = encoder_get_target_rpm(ENCODER_LEFT);
        right_target_rpm = encoder_get_target_rpm(ENCODER_RIGHT);
        left_pid_output = encoder_get_pid_output(ENCODER_LEFT);
        right_pid_output = encoder_get_pid_output(ENCODER_RIGHT);
        left_pwm_output = encoder_get_pwm_output(ENCODER_LEFT);
        right_pwm_output = encoder_get_pwm_output(ENCODER_RIGHT);

#if (CURRENT_OUTPUT_MODE == OUTPUT_MODE_VOFA)
        // vofa+纯数据格式（逗号分隔，9通道）
        sprintf(uart_buffer,
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f\r\n",
            left_target_rpm,    // 通道1: 左轮目标转速
            left_rpm_filtered,  // 通道2: 左轮实际转速
            left_pid_output,    // 通道3: 左轮速度环PID输出
            right_target_rpm,   // 通道4: 右轮目标转速
            right_rpm_filtered, // 通道5: 右轮实际转速
            right_pid_output,   // 通道6: 右轮速度环PID输出
            left_pwm_output,    // 通道7: 左轮PWM输出
            right_pwm_output,   // 通道8: 右轮PWM输出
            turn_pid_output);   // 通道9: 转向环PID输出
#else
        // 调试信息格式
        sprintf(uart_buffer,
            "=== 编码器PID控制状态 (循环: %lu) ===\r\n"
            "左轮 - 目标: %.1f RPM, 实际: %.2f RPM, PID输出: %.2f\r\n"
            "右轮 - 目标: %.1f RPM, 实际: %.2f RPM, PID输出: %.2f\r\n"
            "原始转速 - 左: %.2f RPM, 右: %.2f RPM\r\n"
            "----------------------------------------\r\n",
            loop_count,
            left_target_rpm, left_rpm_filtered, left_pid_output,
            right_target_rpm, right_rpm_filtered, right_pid_output,
            left_rpm_raw, right_rpm_raw);
#endif

        uart_write_string(UART_0, uart_buffer);

        // 每10秒改变一次目标转速进行测试
        loop_count++;
        if(loop_count % 20 == 0) {
            new_target = (loop_count / 20) % 2 == 0 ? 150.0f : 50.0f;
            encoder_set_target_rpm(ENCODER_LEFT, new_target);
            encoder_set_target_rpm(ENCODER_RIGHT, new_target);

#if (CURRENT_OUTPUT_MODE == OUTPUT_MODE_DEBUG)
            sprintf(uart_buffer, "*** 目标转速已更改为: %.1f RPM ***\r\n", new_target);
            uart_write_string(UART_0, uart_buffer);
#endif
        }

        system_delay_ms(20);
    }
}

