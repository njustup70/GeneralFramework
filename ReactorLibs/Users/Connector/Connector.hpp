#pragma once
#include "System.hpp"
#include "bsp_uart.hpp"
#include "bsp_can.hpp"

enum class Type
{
    CONTACTOR, // 触点
    OPTICAL,   // 光通信
    QR_CODE    // 二维码通信
};

enum class Protocol
{
    UART, // 串口，注意框架已经使用了2，3，6串口，不再有可配置接收DMA的串口
    CAN // CAN
};

enum class Role
{
    MASTER, // 只发送
    SLAVE,  // 只接收
    BOTH    // 既可发送也可接收
};

class Connector : public Application
{
    SINGLETON(Connector) : Application("connector") {};
    APPLICATION_OVERRIDE
private:
    // 触点用的串口句柄
    BSP::UART::Handler com_uart_handler; 

    // 请在这里配置通信方式和角色
    Type _type = Type::OPTICAL;
    Protocol _protocol = Protocol::UART;
    Role _role = Role::SLAVE;

    uint8_t _rx_KFS[12] = {0}; // 接收方的KFS值

    // // 启动连接标志
    // bool _enabled = false;

    static void _RxCallback(BSP::UART::UartID huart, uint8_t *rxData, uint8_t size);//属于整个类，而不是某个实例，所以没有this指针，和UART库的C的函数指针兼容

public:
    // void Launch(); // 启动连接器
    // void Close(); // 停止连接器

    // 触点串口接收回调函数
    void SendKFSValue(); // 当作为发送方时，留给遥控器库的KFS值的接口函数
    // uint8_t* GetKFSValue(); // 接收方将KFS值返回，留给外部逻辑调用
};

extern Connector &connector;
