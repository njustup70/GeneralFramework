#ifndef MONITOR_HPP
#define MONITOR_HPP

#include "typeinfo"
#include "std_math.hpp"
#include "stdarg.h"
#include "SysDefs.hpp"
#include "bsp_uart.hpp"

/**
 * @brief 用于监控机器人的各项状态，还有调试、日志等功能]
 * @warning 本类也属于：单例
 */
class Monitor
{
    friend class SystemType;      // 允许系统类访问私有成员
    SINGLETON(Monitor){};

public:
private:
    typedef enum
    {
        track_uint8,
        track_int8,
        track_uint16,
        track_int16,
        track_uint32,
        track_int32,
        track_float,
    } Track_t;

    typedef struct
    {
        void *track_addr; // 变量地址
        void *buf_addr;   // 将其变量值存入的缓存地址
        uint8_t bytes;    // 变量的字节数
    } MonitorLinkage;

    /// @brief 最多追踪32Byte的数据
    byte track_buf[32];
    /// @brief 最多向Vofa发送64Byte数据
    byte vofa_buf[64];

    // 变量的地址和类型存储在这里
    void *track_list[8];
    Track_t track_type[8];

    uint8_t track_count = 0;

    bool high_performance_mode = false;         // 高性能模式标志位（1000Hz跟踪）

public:
    BSP::UART::Handler track_uart;
    BSP::UART::Handler farcon_uart;

public:
    /**
    * @brief 初始化监视器
    * 
    * @param huart_track 用于发送跟踪数据的串口，传入 nullptr 则不启用跟踪功能；
    * 请注意，huart_track 只负责track的数据，日志数据取决于bsp_log的配置！！
    * @param huart_farc 暂未启用
    * @param vofa_mode 
    */
    void Init(BSP::UART::UartID huart_host, BSP::UART::UartID huart_farc, bool vofa_mode = false);

    /**
     * @brief 运行方法
     * @note 该方法应被周期性调用，以处理监控任务
     * @details 内含发送日志信息、发送机器人状态码、监控维护模块等功能
     */
    // void Run();

    /// @brief 切换高性能模式
    void Perflize();

    /// @brief 发送日志
    void LogInfo(const char *format, ...);
    // void LogInfo(const char *format);       // 空参数重载

    /// @brief 发送正常完成日志
    void LogOK(const char *format, ...);

    /// @brief 发送说明性日志
    void LogSpec(const char *format, ...);

    /// @brief 发送响应类日志
    void LogRespond(const char *format, ...);

    /// @brief 发送警告
    void LogWarning(const char *format, ...);
    // void LogWarning(const char *format); // 空参数重载

    /// @brief 发送错误
    void LogError(const char *format, ...);
    // void LogError(const char *format);   // 空参数重载

    /**
     * @brief 跟踪某个变量
     * @note 调用本函数后，其将被编码并加入机器人状态码中
     * @param track_hz 跟踪频率，单位Hz，最大为200Hz
     */
    template <typename T>
    void Track(T &targ)
    {
        using namespace std;

        // 检查追踪越界
        if (track_count >= 8)
        {
            LogError("Monitor: Track list full!\n");
            return;
        }

        // 存储其类型信息
        const type_info &targ_type = typeid(targ);
        if (targ_type == typeid(uint8_t))
        {
            track_type[track_count] = track_uint8;
        }
        else if (targ_type == typeid(int8_t))
        {
            track_type[track_count] = track_int8;
        }
        else if (targ_type == typeid(uint16_t))
        {
            track_type[track_count] = track_uint16;
        }
        else if (targ_type == typeid(int16_t))
        {
            track_type[track_count] = track_int16;
        }
        else if (targ_type == typeid(uint32_t))
        {
            track_type[track_count] = track_uint32;
        }
        else if (targ_type == typeid(int32_t))
        {
            track_type[track_count] = track_int32;
        }
        else if (targ_type == typeid(float))
        {
            track_type[track_count] = track_float;
        }
        else
        {
            LogError("Monitor: Unknown Track type!\n");
            return;
        }

        // 存储其地址
        track_list[track_count] = (void *)&targ;

        track_count++;
    }

    /// @brief 发送跟踪信息
    void TrackLog();

    /**
     * @brief 发送跟踪信息 (JustFloat格式，极快)
     * @note 无论由于Track的是什么类型，这里统统转为float发送
     * @note 协议帧尾: 00 00 80 7f
     */
    void TrackLogJustFloat();
};

namespace APP
{
    extern Monitor& monit;
}

#endif
