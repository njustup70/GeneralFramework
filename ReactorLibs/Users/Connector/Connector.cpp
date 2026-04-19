// 26赛季用于两个机器人之间通信
#include "Connector.hpp"
#include "farcon.hpp"

#define FRAME_HEAD 0x5A
#define FRAME_TYPE 0xAA // 此类型为KFS数据包
// static uint8_t _rx_KFS[12] = {0}; // 接收方的KFS值
Connector &connector = Connector::GetInstance();

void Connector::Start()
{
    switch (_type)
    {
        case Type::CONTACTOR:
            // 触点通信
            if (_protocol == Protocol::UART)
            {
                com_uart_handler = BSP::UART::Apply(Hardware::huart_host);
                if(_role == Role::SLAVE)
                {
                    // 接收方
                    com_uart_handler.RegisterRx(14, _RxCallback);
                }
            }
            else if (_protocol == Protocol::CAN)
            {
                // CAN
            }
            break;

        case Type::OPTICAL:
            // 光通信，用串口
            com_uart_handler = BSP::UART::Apply(Hardware::huart_host);
            if(_role == Role::SLAVE)
            {
                // 接收方
                com_uart_handler.RegisterRx(14, _RxCallback);
            }
            break;

        case Type::QR_CODE:
            // 二维码通信，用上位机串口
            break;

        default:
            break;
    }
}

void Connector::Update()
{
}
// void Connector::Launch()
// {
//     _enabled = true;
// }

// void Connector::Close()
// {
//     _enabled = false;
// }

//uint8_t KFS[14] = {FRAME_HEAD, FRAME_TYPE}; // 存储KFS信息的帧
void Connector::SendKFSValue()
{
    // 发送KFS值
    uint8_t KFS[14] = {FRAME_HEAD, FRAME_TYPE}; // 存储KFS信息的帧
    for(int i = 0; i < 12; i++)
    {
        KFS[i+2] = farcon.KFS_values[i];
    }
    if(_protocol == Protocol::UART)
    {
        // 发送KFS值
        com_uart_handler.Transmit(KFS, 14);
    }
}

void Connector::_RxCallback(BSP::UART::UartID huart, uint8_t *rxData, uint8_t size)
{
    // 处理串口触点接收数据
    if(rxData[0] == FRAME_HEAD && rxData[1] == FRAME_TYPE)
    {
        for(int i = 0; i < 12; i++)
        {
            Connector::GetInstance()._rx_KFS[i] = rxData[i+2];
        }
    }
}

// uint8_t* Connector::GetKFSValue()
// {
//     return _rx_KFS;
// }