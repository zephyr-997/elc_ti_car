/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          zf_device_oled
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
* 2023-04-28       pudding            增加中文注释说明
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                  ------------------------------------
*                  模块管脚             单片机管脚
*                  D0                 查看 zf_device_oled.h 中 OLED_D0_PIN 宏定义
*                  D1                 查看 zf_device_oled.h 中 OLED_D1_PIN 宏定义
*                  RES                查看 zf_device_oled.h 中 OLED_RES_PIN 宏定义
*                  DC                 查看 zf_device_oled.h 中 OLED_DC_PIN 宏定义
*                  CS                 查看 zf_device_oled.h 中 OLED_CS_PIN 宏定义
*                  VCC                3.3V电源
*                  GND                电源地
*                  ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_oled_h_
#define _zf_device_oled_h_

#include "zf_common_typedef.h"

//===================================================定义 OLED 基本配置=================================================
#define OLED_USE_SOFT_SPI               (0 )                                    // 默认使用硬件 SPI 方式驱动 建议使用硬件 SPI 方式驱动
#if OLED_USE_SOFT_SPI                                                           // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的
//====================================================软件 SPI 驱动====================================================
#define OLED_SOFT_SPI_DELAY             ( 1 )                                   // 软件 SPI 的时钟延时周期 数值越小 SPI 通信速率越快
#define OLED_D0_PIN                     ( A12 )                                 // 软件 SPI SCK 引脚
#define OLED_D1_PIN                     ( A9 )                                  // 软件 SPI MOSI 引脚
//====================================================软件 SPI 驱动====================================================
#else
//====================================================硬件 SPI 驱动====================================================
#define OLED_SPI_SPEED                  ( 30 * 1000 * 1000 )                       // 硬件 SPI 速率
#define OLED_SPI                        ( SPI_0            )                       // 硬件 SPI 号
#define OLED_D0_PIN                     ( SPI0_SCK_A12      )                      // 硬件 SPI SCK 引脚
#define OLED_D1_PIN                     ( SPI0_MOSI_A9     )                       // 硬件 SPI MOSI 引脚
#define OLED_D1_PIN_IN                  (SPI_MISO_NULL	)                          // 硬件 SPI MISO 引脚  OLED没有MISO引脚，但是这里任然需要定义，在spi的初始化时需要使用
//====================================================硬件 SPI 驱动====================================================
#endif


#define OLED_RES_PIN                    (A7)                                 // 液晶复位引脚定义
#define OLED_DC_PIN                     (A15 )                                // 液晶命令位引脚定义
#define OLED_CS_PIN                     (A8 )                                // CS 片选引脚
#define OLED_BRIGHTNESS                 (0x7f)                                  // 设置OLED亮度 越大越亮 范围0-0XFF
#define OLED_DEFAULT_DISPLAY_DIR        (OLED_PORTAIT )                        // 默认的显示方向
#define OLED_DEFAULT_DISPLAY_FONT       (OLED_6X8_FONT )                        // 默认的字体模式
#define OLED_X_MAX                      (128)                                   // 屏幕X轴像素最大值
#define OLED_Y_MAX                      (64 )                                   // 屏幕Y轴像素最大值

#define OLED_RES(x)                     ((x) ? (gpio_high(OLED_RES_PIN)) : (gpio_low(OLED_RES_PIN)))
#define OLED_DC(x)                      ((x) ? (gpio_high(OLED_DC_PIN))  : (gpio_low(OLED_DC_PIN)))
#define OLED_CS(x)                      ((x) ? (gpio_high(OLED_CS_PIN))  : (gpio_low(OLED_CS_PIN)))
//===================================================定义 OLED 基本配置=================================================


//===================================================定义 OLED 参数结构体=================================================
typedef enum
{
    OLED_PORTAIT                        = 0,                                    // 竖屏模式
    OLED_PORTAIT_180                    = 1,                                    // 竖屏模式  旋转180
}oled_dir_enum;

typedef enum
{
    OLED_6X8_FONT                       = 0,                                    // 6x8      字体
    OLED_8X16_FONT                      = 1,                                    // 8x16     字体
    OLED_16X16_FONT                     = 2,                                    // 16x16    字体 目前不支持
}oled_font_size_enum;
//===================================================定义 OLED 基本配置=================================================


//===================================================声明 OLED 基本函数=================================================
void    oled_clear                      (void);                                                             // OLED 清屏函数
void    oled_full                       (const uint8 color);                                                // OLED 屏幕填充函数
void    oled_set_dir                    (oled_dir_enum dir);                                                // OLED 设置显示方向
void    oled_set_font                   (oled_font_size_enum font);                                         // OLED 设置显示字体
void    oled_draw_point                 (uint16 x, uint16 y, const uint8 color);                            // OLED 画点函数

void    oled_show_string                (uint16 x, uint16 y, const char ch[]);                              // OLED 显示字符串
void    oled_show_int                   (uint16 x, uint16 y, const int32 dat, uint8 num);                   // OLED 显示32位有符号 (去除整数部分无效的0)
void    oled_show_uint                  (uint16 x, uint16 y, const uint32 dat, uint8 num);                  // OLED 显示32位无符号 (去除整数部分无效的0)
void    oled_show_float                 (uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum);

void    oled_show_binary_image          (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);                     // OLED 显示二值图像 数据每八个点组成一个字节数据
void    oled_show_gray_image            (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 threshold);    // OLED 显示 8bit 灰度图像 带二值化阈值

void    oled_show_wave                  (uint16 x, uint16 y, const uint16 *image, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max);              // OLED 显示波形
void    oled_show_chinese               (uint16 x, uint16 y, uint8 size, const uint8 *chinese_buffer, uint8 number);                                                    // OLED 汉字显示
void    oled_init                       (void);                                                             // OLED 初始化函数
//===================================================定义 OLED 基本函数=================================================


//===================================================定义 OLED 扩展配置=================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 128*64 显示小钻风图像
// 参数说明     p               图像数组
// 返回参数     void
// 使用示例     oled_displayimage7725(ov7725_image_binary[0]);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
#define oled_displayimage7725(p)        (oled_show_binary_image(0, 0, (p), OV7725_W, OV7725_H, 128, 64))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 128*64 显示总钻风图像 带二值化
// 参数说明     p               图像数组
// 参数说明     x               二值化显示阈值
// 返回参数     void
// 使用示例     oled_displayimage03x(mt9v03x_image[0], 127);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
#define oled_displayimage03x(p,x)       (oled_show_gray_image(0, 0, (p), MT9V03X_W, MT9V03X_H, 128, 64, (x)))
//===================================================定义 OLED 扩展配置=================================================


#endif
