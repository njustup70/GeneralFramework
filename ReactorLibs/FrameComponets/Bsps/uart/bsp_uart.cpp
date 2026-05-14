#include "bsp_uart.hpp"
#include "bsp_halport.hpp"
#include "bsp_log.hpp"
#include "string.h"

using namespace BSP::UART;

::__UART_HandleTypeDef* BSP::UART::ToHalHandle(UartID id) {
    return reinterpret_cast<UART_HandleTypeDef*>(id);
}

UartID BSP::UART::ToUartID(::__UART_HandleTypeDef* huart) {
    return reinterpret_cast<UartID>(huart);
}

/**
 * @brief 提取文件名辅助函数
 */
static inline const char* GetFileName(const char* path) {
    const char* filename = path;
    for (const char* p = path; *p; ++p) {
        if (*p == '/' || *p == '\\') {
            filename = p + 1;
        }
    }
    return filename;
}

/// @brief 最大实例数量，取决于实际使用的 UART 数
static constexpr uint8_t max_bspuart_inst_nums = 6;

/// @brief 用于存储 UART 实例的静态数组和计数器
static Instance insts[max_bspuart_inst_nums];

/// @brief 已经创建的实例数量
static uint8_t instance_count = 0;

/**
 * @brief 根据 UART 句柄查找对应的实例
 */
static Instance *FindInstByHuart(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return nullptr;
    }

    for (uint8_t i = 0; i < instance_count; i++)
    {
        if (insts[i].IsUsing(ToUartID(huart)))
        {
            return &insts[i];
        }
    }

    return nullptr;
}

/**
 * @brief 申请一个默认 DMA 发送模式的 UART 实例句柄
 */
Handler BSP::UART::Apply(UartID id, const char* file, int line)
{
    return Apply(id, TxMode::Dma, file, line);
}

/**
 * @brief 按指定发送模式申请一个 UART 实例句柄
 */
Handler BSP::UART::Apply(UartID id, TxMode tx_mode, const char* file, int line)
{
    UART_HandleTypeDef *huart = ToHalHandle(id);
    
    // 确保不是空的 UART 句柄
    if (huart == NULL)
    {
        BspLog_LogError("[Bsp] Empty UartHandle When Apply! [%s:%d]\n", GetFileName(file), line);
        return Handler(nullptr);
    }

    // 看看是否已经存在该Uart的实例
    Instance *exist_inst = FindInstByHuart(huart);

    // 如果已经存在，直接返回对应的 Handler
    if (exist_inst != nullptr)
    {
        // 同一硬件串口的发送模式以首次申请为准，避免运行期切换破坏状态机
        if (exist_inst->GetTxMode() != tx_mode)
        {
            BspLog_LogWarning("[Bsp] Uart TxMode Already Fixed, Ignore New TxMode! [%s:%d]\n", GetFileName(file), line);
        }
        return Handler(exist_inst);
    }

    // 如果实例数量已达上限，无法创建新实例
    if (instance_count >= max_bspuart_inst_nums)
    {
        BspLog_LogError("[Bsp] Too Many Uart Instance! [%s:%d]\n", GetFileName(file), line);
        return Handler(nullptr);
    }
    
    // 初始化新实例
    insts[instance_count].Init(id, tx_mode);

    // 返回新实例的 Handler
    Instance *new_inst = &insts[instance_count];
    instance_count++;
    return Handler(new_inst);
}

Handler::Handler(Instance *inst) : instance(inst)
{
}

void Handler::Transmit(const uint8_t *data, uint16_t len)
{
    // 确保不是无效调用
    if (instance == nullptr)
    {
        BspLog_LogWarning("[Bsp] Invalid Transmit, Empty Handler!\n");
        return;
    }

    instance->Transmit(data, len);
}

bool Handler::RegisterRx(uint16_t rx_setlen, RxCallback rx_callback, const char* file, int line)
{
    // 确保不是无效调用
    if (instance == nullptr)
    {
        BspLog_LogError("[Bsp] Invalid RxRegist, Empty Handler! [%s:%d]\n", GetFileName(file), line);
        return false;
    }

    return instance->RegisterRx(rx_setlen, rx_callback, file, line);
}

bool Handler::IsValid() const
{
    return (instance != nullptr);
}

/***---------    实例部分接口    ---------***/

Instance::Instance()
{
    Init(nullptr);
}

Instance::Instance(UartID id, TxMode tx_mode)
{
    Init(id, tx_mode);
}

/**
 * @brief 初始化实例，重置所有成员变量
 */
void Instance::Init(UartID in_id, TxMode tx_mode)
{
    this->id = in_id;
    this->tx_mode_ = tx_mode;
    this->rx_callback = nullptr;
    this->rx_setlen_ = 0;
    this->rx_registered_ = 0;

    this->tx_fifo_.head = 0;
    this->tx_fifo_.tail = 0;
    this->tx_fifo_.sending_len = 0;
    this->tx_fifo_.is_busy = 0;

    memset(this->tx_fifo_.buffer, 0, sizeof(this->tx_fifo_.buffer));
    memset(this->rx_buffer_, 0, sizeof(this->rx_buffer_));
}


bool Instance::IsUsing(UartID target_id) const
{
    return (this->id != nullptr && this->id == target_id);
}

/**
 * @brief 获取实例首次申请时固定的发送模式
 */
TxMode Instance::GetTxMode() const
{
    return this->tx_mode_;
}

void Instance::Transmit(const uint8_t *data, uint16_t len)
{
    // 确保不是空调用
    if (this->id == nullptr)
    {
        BspLog_LogWarning("[Bsp] Invalid Transmit, Empty Instance!\n");
        return;
    }
    if (data == NULL)
    {
        BspLog_LogWarning("[Bsp] Invalid Transmit, Empty Data Pointer!\n");
        return;
    }
    if (len == 0)
    {
        BspLog_LogWarning("[Bsp] Invalid Transmit, Zero Length!\n");
        return;
    }

    // 将希望发送的数据写入FIFO缓冲区
    for (uint16_t i = 0; i < len; i++)
    {
        // 计算下一个头部位置，检查是否会与尾部重叠（即缓冲区满）
        uint16_t next_head = (uint16_t)((this->tx_fifo_.head + 1U) % BSP_UART_TX_BUF_SIZE);

        // 如果下一个头部位置与尾部重叠，说明缓冲区已满，无法继续写入
        if (next_head == this->tx_fifo_.tail)
        {
            BspLog_LogWarning("[Bsp] UART Tx FIFO Full, Data Lost!\n");
            break;
        }

        // 写入数据并更新头部位置
        this->tx_fifo_.buffer[this->tx_fifo_.head] = data[i];
        this->tx_fifo_.head = next_head;
    }

    // 按当前模式启动发送（如果当前发送空闲）
    this->TryStartTx();
}

/**
 * @brief 注册接收回调函数，并启动DMA接收
 * @param rx_setlen 期望接收数据长度，也是DMA的最大长度
 * @param rx_callback 用户定义的接收回调函数
 */
bool Instance::RegisterRx(uint16_t rx_setlen, RxCallback rx_callback, const char* file, int line)
{
    // 确保参数有效
    if (this->id == nullptr)
    {
        BspLog_LogWarning("[Bsp] RxRegist Failed, Empty Instance! [%s:%d]\n", GetFileName(file), line);
        return false;
    }
    if (rx_callback == nullptr)
    {
        BspLog_LogWarning("[Bsp] RxRegist Failed, Empty Callback! [%s:%d]\n", GetFileName(file), line);
        return false;
    }
    if (rx_setlen == 0)
    {
        BspLog_LogWarning("[Bsp] RxRegist Failed, Zero Length! [%s:%d]\n", GetFileName(file), line);
        return false;
    }

    // 每个实例只允许注册一次接收回调
    if (this->rx_registered_ != 0)
    {
        BspLog_LogWarning("[Bsp] RxRegist Failed, This Uart Already has RxCallback! [%s:%d]\n", GetFileName(file), line);
        return false;
    }

    // 限制设置的接收长度不能超过缓冲区大小
    if (rx_setlen > BSP_UART_RX_BUF_SIZE)
    {
        rx_setlen = BSP_UART_RX_BUF_SIZE;
    }

    // 保存回调函数和期望接收长度，并标记为已注册
    this->rx_callback = rx_callback;
    this->rx_setlen_ = rx_setlen;
    this->rx_registered_ = 1;

    // 启动DMA接收
    memset(this->rx_buffer_, 0, sizeof(this->rx_buffer_));

    if (HAL_UARTEx_ReceiveToIdle_DMA(ToHalHandle(this->id), this->rx_buffer_, this->rx_setlen_) != HAL_OK)
    {
        BspLog_LogError("[Bsp] Try Enable HAL_IdleRxCallback_DMA, but Failed!  [%s:%d]\n", GetFileName(file), line);
        this->rx_registered_ = 0;
        this->rx_setlen_ = 0;
        this->rx_callback = nullptr;
        return false;
    }

    // 关闭DMA半满中断，避免在接收过程中被过早触发
    UART_HandleTypeDef* h = ToHalHandle(this->id);
    if (h->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(h->hdmarx, DMA_IT_HT);
    }

    return true;
}

/**
 * @brief 按当前发送模式尝试启动发送
 */
void Instance::TryStartTx()
{
    if (this->tx_mode_ == TxMode::Interrupt)
    {
        this->TryStartTxIT();
        return;
    }

    this->TryStartTxDMA();
}

/**
 * @brief 尝试启动DMA发送，如果当前DMA空闲且FIFO中有数据
 */
void Instance::TryStartTxDMA()
{
    if (this->id == nullptr)
    {
        BspLog_LogWarning("[Bsp] Unable to Start Tx DMA, Empty Instance!\n");
        return;
    }

    if (this->tx_fifo_.is_busy != 0)
    {
        // DMA正在发送中，等待当前发送完成后会自动检查并继续发送剩余数据
        return;
    }

    if (this->tx_fifo_.head == this->tx_fifo_.tail)
    {
        // FIFO中没有数据需要发送
        return;
    }

    // 计算本次最大连续发送长度（处理环形回绕）
    uint16_t send_len = 0;

    // 如果头部在尾部前面，直接计算差值
    if (this->tx_fifo_.head > this->tx_fifo_.tail)
    {
        send_len = this->tx_fifo_.head - this->tx_fifo_.tail;
    }
    // 如果头部在尾部后面，说明环形回绕了，需要计算从尾部到缓冲区末尾的长度
    else
    {
        send_len = BSP_UART_TX_BUF_SIZE - this->tx_fifo_.tail;
    }

    // 启动DMA发送
    this->tx_fifo_.sending_len = send_len;

    if (HAL_UART_Transmit_DMA(ToHalHandle(this->id), &this->tx_fifo_.buffer[this->tx_fifo_.tail], send_len) == HAL_OK)
    {
        this->tx_fifo_.is_busy = 1;
    }
    else
    {
        BspLog_LogError("[Bsp] Failed to Start HAL_Tx_DMA!\n");
        this->tx_fifo_.sending_len = 0;
    }
}

/**
 * @brief 尝试启动TX中断发送，如果当前发送空闲且FIFO中有数据
 */
void Instance::TryStartTxIT()
{
    if (this->id == nullptr)
    {
        BspLog_LogWarning("[Bsp] Unable to Start Tx IT, Empty Instance!\n");
        return;
    }

    if (this->tx_fifo_.is_busy != 0)
    {
        // 当前发送未完成，等待完成回调继续推进FIFO
        return;
    }

    if (this->tx_fifo_.head == this->tx_fifo_.tail)
    {
        // FIFO为空，无需启动新的发送事务
        return;
    }

    // 计算本次最大连续发送长度，避免一次发送跨越环形缓冲区尾部
    uint16_t send_len = 0;
    if (this->tx_fifo_.head > this->tx_fifo_.tail)
    {
        send_len = this->tx_fifo_.head - this->tx_fifo_.tail;
    }
    else
    {
        send_len = BSP_UART_TX_BUF_SIZE - this->tx_fifo_.tail;
    }

    // 启动UART TX中断发送，完成后由HAL_UART_TxCpltCallback继续推进
    this->tx_fifo_.sending_len = send_len;

    if (HAL_UART_Transmit_IT(ToHalHandle(this->id), &this->tx_fifo_.buffer[this->tx_fifo_.tail], send_len) == HAL_OK)
    {
        this->tx_fifo_.is_busy = 1;
    }
    else
    {
        BspLog_LogError("[Bsp] Failed to Start HAL_Tx_IT!\n");
        this->tx_fifo_.sending_len = 0;
    }
}

/**
 * @brief HAL UART 发送完成回调入口，更新FIFO状态并尝试发送剩余数据
 */
void Instance::OnTxCplt()
{
    // 确保不是空调用
    if (this->id == nullptr)
    {
        BspLog_LogError("[Bsp] Meet Empty UartInst in TxCplt Callback!\n");
        return;
    }

    // 更新FIFO状态，释放已发送的数据空间
    this->tx_fifo_.tail = (uint16_t)((this->tx_fifo_.tail + this->tx_fifo_.sending_len) % BSP_UART_TX_BUF_SIZE);
    this->tx_fifo_.sending_len = 0;

    // 检查FIFO中是否还有数据需要发送，如果有则继续发送
    if (this->tx_fifo_.head == this->tx_fifo_.tail)
    {
        this->tx_fifo_.is_busy = 0;
        return;
    }

    // 释放忙标志，让统一入口按当前模式启动下一段发送
    this->tx_fifo_.is_busy = 0;
    this->TryStartTx();
}

void Instance::OnRxEvent(uint16_t size)
{
    if (this->id == nullptr || this->rx_registered_ == 0 || this->rx_callback == nullptr)
    {
        return;
    }

    uint16_t real_size = size;
    if (real_size > this->rx_setlen_)
    {
        real_size = this->rx_setlen_;
    }

    this->rx_callback(this->id, this->rx_buffer_, real_size);

    memset(this->rx_buffer_, 0, this->rx_setlen_);
    UART_HandleTypeDef* h = ToHalHandle(this->id);
    HAL_UARTEx_ReceiveToIdle_DMA(h, this->rx_buffer_, this->rx_setlen_);
    if (h->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(h->hdmarx, DMA_IT_HT);
    }
}



/****-------    覆写HAL库的中断函数     -------****/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Instance *inst = FindInstByHuart(huart);
    if (inst == nullptr)
    {
        return;
    }

    inst->OnRxEvent(Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Instance *inst = FindInstByHuart(huart);
    if (inst == nullptr)
    {
        return;
    }

    inst->OnTxCplt();
}
