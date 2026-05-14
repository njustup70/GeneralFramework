#pragma once

#define USE_REAL_HAL

/**
 * @warning 本文件是 HAL 库的接口文件，因此，为避免污染，不应该被任何hpp文件引用
 *      BSP层中的引用全部发生在cpp文件中，这可以有效将 `Mod` 层以及上与 `HAL` 隔离解耦
 */

/* ==================================================================
 * 场景 1：在 Main 分支（真实战场），有 CubeMX 环境
 * ================================================================== */
#ifdef USE_REAL_HAL  // 这个宏可以在 main.h 或 编译选项里定义

    // 包含真的 HAL 库
    #include "main.h" 
    #include "can.h"
    #include "usart.h"
    #include "dma.h"
    #include "spi.h"

/* ==================================================================
 * 场景 2：在 Pure-Fram 分支（实验室），无硬件环境
 * ================================================================== */
#else 

    #include <stdint.h>
    #include <string.h> // for memset

    typedef void CAN_HandleTypeDef;
    typedef void CAN_RxHeaderTypeDef;
    typedef void CAN_FilterTypeDef;

    typedef void UART_HandleTypeDef;

    typedef void TIM_HandleTypeDef;

    typedef void GPIO_TypeDef;

    // 这种写法解决报错
    #define REACTOR_UART_SEND(...)      ((void)0) 
    #define REACTOR_CAN_FILTER_CFG(...) ((void)0)
    #define REACTOR_DMA_DISABLE_IT(...) ((void)0)


    // 补充缺失的枚举值
    #define DMA_IT_HT 0x01
    #define CAN_RX_FIFO0 0x00

    #ifndef __weak
    #if defined ( __GNUC__ )
        #define __weak   __attribute__((weak))
    #elif defined ( __CC_ARM )
        #define __weak   __weak
    #endif
    #endif

#endif 
 
namespace BSP
{
    namespace CAN { struct OpaqueCan; using CanID = OpaqueCan*; }
    namespace UART { struct OpaqueUart; using UartID = OpaqueUart*; }
    namespace SPI { struct OpaqueSpi; using SpiID = OpaqueSpi*; }
}

// ---- 框架助手函数：将 HAL 句柄转换为框架 ID ----
inline BSP::CAN::CanID ToID(CAN_HandleTypeDef* handle)
{
    return reinterpret_cast<BSP::CAN::CanID>(handle);
}

inline BSP::UART::UartID ToID(UART_HandleTypeDef* handle)
{
    return reinterpret_cast<BSP::UART::UartID>(handle);
}

inline BSP::SPI::SpiID ToID(SPI_HandleTypeDef* handle)
{
    return reinterpret_cast<BSP::SPI::SpiID>(handle);
}


namespace Hardware
{
    void Config_Hardwares();
    void Config_Parameters();

    extern float MainFreq_MHz;

    /// @note 如果本标志位被激活，日志会被同步发送到 UART_HOST
    extern bool RTTLogAtUart;
    extern bool RTTLogUartItrp;

    /***---------------     框架定时器    ---------------***/
    /// @brief WS2812灯带定时器
    extern TIM_HandleTypeDef* htim_led;
};
