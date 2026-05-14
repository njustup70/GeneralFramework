#pragma once
#include <cstdint>

/*
 * @brief 发送缓冲区大小
 * 控制发送环形缓冲区的大小。
 * 超过此大小 + 当前正在发送的数据将被丢弃或根据实现被阻塞（此处为丢弃）。
 */
#define BSP_UART_TX_BUF_SIZE 2048
#define BSP_UART_RX_BUF_SIZE 64

struct __UART_HandleTypeDef;

namespace BSP
{
    namespace UART
    {
        // 用于触发C++类型检查，防止空指针谬误
        struct OpaqueUart;
        
        /// @brief 不透明指针 UART_HandleTypeDef* 的框架解耦替代
        using UartID = OpaqueUart*; 
        
        // 预定义实例
        class Instance;
        class Handler;

        // 接收回调函数类型
        using RxCallback = void (*)(UartID id, uint8_t *rxData, uint8_t size);

        /// @brief UART 发送后端模式
        enum class TxMode : uint8_t
        {
            Dma,       ///< 使用 DMA + FIFO 异步发送
            Interrupt  ///< 使用 UART TX 中断 + FIFO 异步发送
        };

        ::__UART_HandleTypeDef* ToHalHandle(UartID id);
        UartID ToUartID(::__UART_HandleTypeDef* huart);

        /// @brief 申请一个 UART 实例
        Handler Apply(UartID id, const char* file = __builtin_FILE(), int line = __builtin_LINE());
        Handler Apply(UartID id, TxMode tx_mode, const char* file = __builtin_FILE(), int line = __builtin_LINE());

        class Handler
        {
        public:
            Handler(Instance *inst = nullptr);

            /// @brief 发送数据（DMA + FIFO）
            void Transmit(const uint8_t *data, uint16_t len);

            /// @brief 注册接收回调（实例级，仅允许一次）
            bool RegisterRx(uint16_t rx_setlen, RxCallback rx_callback, const char* file = __builtin_FILE(), int line = __builtin_LINE());

            bool IsValid() const;
            
            Instance *instance;
        };

        /**
         * @brief UART 实例类
         * @note 每个实例唯一对应一个硬件 UART
         */
        class Instance
        {
        public:
            /*---   接口    ---*/
            Instance();

            /// @brief 构造函数
            /// @param id 目标硬件串口层的不透明句柄
            explicit Instance(UartID id, TxMode tx_mode = TxMode::Dma);

            /// @brief 重置并初始化实例
            void Init(UartID id, TxMode tx_mode = TxMode::Dma);

            /// @brief 发送串口数据（带缓冲，默认DMA）
            void Transmit(const uint8_t *data, uint16_t len);

            /// @brief 注册接收回调（实例级，仅允许一次）
            bool RegisterRx(uint16_t rx_setlen, RxCallback rx_callback, const char* file = __builtin_FILE(), int line = __builtin_LINE());

            bool IsUsing(UartID target_id) const;

            /// @brief 获取本实例发送模式
            TxMode GetTxMode() const;

            /// @brief HAL UART 发送完成回调入口（DMA/IT共用）
            void OnTxCplt();

            /// @brief HAL UART DMA接收事件回调入口
            void OnRxEvent(uint16_t size);

            /*---   成员变量    ---*/
            /// @brief 本实例对应的硬件 UART 的模糊句柄
            UartID id;

            /// @brief 本实例对应的接收回调函数
            RxCallback rx_callback;

        private:
            struct TxFIFO
            {
                uint8_t buffer[BSP_UART_TX_BUF_SIZE];
                volatile uint16_t head;
                volatile uint16_t tail;
                volatile uint16_t sending_len;
                volatile uint8_t is_busy;
            };

            /// @brief 按发送模式分流启动发送
            void TryStartTx();

            /// @brief 尝试启动 DMA 发送
            void TryStartTxDMA();

            /// @brief 尝试启动 TX 中断发送
            void TryStartTxIT();

            TxMode tx_mode_;
            TxFIFO tx_fifo_;
            uint8_t rx_buffer_[BSP_UART_RX_BUF_SIZE];
            uint16_t rx_setlen_;
            uint8_t rx_registered_;
        };
    }
    
}
