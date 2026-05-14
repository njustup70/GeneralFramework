# BSP UART 库使用说明

`bsp_uart` 是一个基于 STM32 HAL 库封装的串口通信库，通过 **DMA/中断 + FIFO** 实现异步发送，并利用 **IDLE (空闲中断) + DMA** 实现变长数据接收。

## 核心功能
- **异步发送**：数据先写入 FIFO，再由 DMA 或 UART TX 中断发送，不阻塞应用层。
- **高效接收**：自动触发 IDLE 中断，支持变长数据包接收。
- **解耦设计**：使用 `UartID` (不透明指针) 替代 HAL 句柄，减少头文件依赖。

## 架构说明
- **解耦设计**：`Bsp` 层不直接面对 `HAL`，而是通过库文件 `halport` 作为统一的代理接口。
- **类型遮蔽**：上层通过 `Hardware` 命名空间访问串口实例（如 `Hardware::huart_host`），底层自动处理其与 HAL 句柄的映射，实现框架层与硬件层的深度解耦。


## 快速上手

> **注意**：通常情况下，`Bsp` 层的库应在 `Mod` 及以上的层调用。

### 获取串口句柄 (`Handler`)
在上层代码中，直接使用 `Hardware` 命名空间预定义的 `UartID` 实例来申请 `Handler`。
```cpp
#include "bsp_uart.hpp"

// 示例：获取工控机通信串口的句柄
auto uart_handler = BSP::UART::Apply(Hardware::huart_host);

```
如果某个串口没有配置 `DMA`，也可以使用 **中断模式** 发送  
使用前，请确认已经在 CubeMX 中正确配置了对应串口的全局中断

```cpp
// 没有配置 TX DMA 的串口，可使用 TX 中断发送
auto uart_it_handler = BSP::UART::Apply(Hardware::huart_other, BSP::UART::TxMode::Interrupt);
```

### 发送数据 (`Transmit`)
直接调用 `Transmit` 即可。库会自动处理 FIFO 缓冲，并按实例发送模式启动 DMA 或 TX 中断。
```cpp
uint8_t data[] = {0x01, 0x02, 0x03};
uart_handler.Transmit(data, sizeof(data));
```

### 注册并接收数据 (`RegisterRx`)
定义一个回调函数，并将其注册到串口句柄中。

> 截止 2026-5-4，接收回调仍只支持 DMA 模式

```cpp
// 定义回调函数
void MyRxCallback(BSP::UART::UartID id, uint8_t *rxData, uint8_t size) {
    // 处理接收到的数据 (rxData)
}

// 注册回调（设置最大单次接收长度）
uart_handler.RegisterRx(64, MyRxCallback);
```

---

## 注意事项

- **单实例原则**：每个硬件串口只能申请一个 `Instance`。
- **发送模式**：默认使用 `TxMode::Dma`；同一串口首次 `Apply` 时确定发送模式，后续重复申请不会动态切换。
- **回调限制**：每个实例仅允许注册 **一次** 接收回调。
- **缓冲区大小**：
  - 发送 FIFO 默认为 `2048` 字节 (`BSP_UART_TX_BUF_SIZE`)。
  - 接收单次最大包长不应超过 `64` 字节 (`BSP_UART_RX_BUF_SIZE`)。
- **常见问题**：
  - 若日志出现 `[Bsp] UART Tx FIFO Full`，说明发送频率过高或波特率过低。
  - DMA 发送模式需要开启对应串口的 **TX DMA**。
  - `TxMode::Interrupt` 发送模式需要开启对应串口的 **USART 全局中断**。
  - 接收功能仍需要开启对应串口的 **DMA 接收** 和 **USART 全局中断**。
