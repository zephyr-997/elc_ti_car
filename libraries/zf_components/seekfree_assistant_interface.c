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
* 文件名称          seekfree_assistant_interface
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2025-06-1        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_typedef.h"
#include "zf_common_fifo.h"
#include "zf_common_debug.h"
#include "zf_driver_uart.h"
#include "zf_device_wireless_uart.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wifi_spi.h"
#include "seekfree_assistant.h"

#include "seekfree_assistant_interface.h"


extern seekfree_assistant_transfer_callback_function   seekfree_assistant_transfer_callback;    // 数据发送函数指针
extern seekfree_assistant_receive_callback_function    seekfree_assistant_receive_callback;     // 数据接收函数指针


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手发送函数
// 参数说明     *buff           需要发送的数据地址
// 参数说明     length          需要发送的长度
// 返回参数     uint32          剩余未发送数据长度
// 使用示例
//-------------------------------------------------------------------------------------------------------------------
ZF_WEAK uint32 seekfree_assistant_transfer (const uint8 *buff, uint32 length)
{
    
    // 当选择自定义通讯方式时 需要自行完成数据发送功能
    return length;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手接收数据函数
// 参数说明     *buff           需要接收的数据地址
// 参数说明     length          要接收的数据最大长度
// 返回参数     uint32          接收到的数据长度
// 使用示例
//-------------------------------------------------------------------------------------------------------------------
ZF_WEAK uint32 seekfree_assistant_receive (uint8 *buff, uint32 length)
{
    // 当选择自定义通讯方式时 需要自行完成数据接收功能
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐飞助手接口 初始化
// 参数说明
// 返回参数     void
// 使用示例     seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI); 使用高速WIFI SPI模块进行数据收发
// 备注         需要自行调用设备的初始化，例如使用无线转串口进行数据的收发，则需要自行调用无线转串口的初始化，然后再调用seekfree_assistant_interface_init完成逐飞助手的接口初始化
//-------------------------------------------------------------------------------------------------------------------
ZF_WEAK void seekfree_assistant_interface_init (seekfree_assistant_transfer_device_enum transfer_device)
{
    switch(transfer_device)
    {
        case SEEKFREE_ASSISTANT_DEBUG_UART:
        {
            seekfree_assistant_transfer_callback = debug_send_buffer;
            seekfree_assistant_receive_callback = debug_read_ring_buffer;
        }break;
        
        case SEEKFREE_ASSISTANT_WIRELESS_UART:
        {
            seekfree_assistant_transfer_callback = wireless_uart_send_buffer;
            seekfree_assistant_receive_callback = wireless_uart_read_buffer;
        }break;
        
//        case SEEKFREE_ASSISTANT_CH9141:
//        {
//            seekfree_assistant_transfer_callback = bluetooth_ch9141_send_buffer;
//            seekfree_assistant_receive_callback = bluetooth_ch9141_read_buffer;
//        }break;
        
        case SEEKFREE_ASSISTANT_WIFI_UART:
        {
            seekfree_assistant_transfer_callback = wifi_uart_send_buffer;
            seekfree_assistant_receive_callback = wifi_uart_read_buffer;
        }break;
        
        case SEEKFREE_ASSISTANT_WIFI_SPI:
        {
            seekfree_assistant_transfer_callback = wifi_spi_send_buffer;
            seekfree_assistant_receive_callback = wifi_spi_read_buffer;
        }break;
        
//        case SEEKFREE_ASSISTANT_BLE6A20:
//        {
//            seekfree_assistant_transfer_callback = ble6a20_send_buffer;
//            seekfree_assistant_receive_callback = ble6a20_read_buffer;
//        }break;
        
        case SEEKFREE_ASSISTANT_CUSTOM:
        {         
            // 根据自己的需求 自行实现seekfree_assistant_transfer与seekfree_assistant_receive函数，完成数据的收发
            
        }break;
    }
}