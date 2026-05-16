#pragma once
#include "led_ws2812.hpp"
#include "odo_ops.hpp"
#include "motor_dji.hpp"
// #include "motor_dm.hpp"
#include "bsp_dwt.hpp"
#include "StateCore.hpp"
#include "arm_math.h"
#include "std_math.hpp"
#include "cstdarg"
#include "cstdio"
#include "SysDefs.hpp"
#include "std_cpp.h"
#include "typeinfo"
#include "Monitor.hpp"
#include "bsp_hardware.hpp"

namespace Systems
{
    // 区分红方蓝方
    const static uint8_t Camp_Blue = 0;
    const static uint8_t Camp_Red = 1;

    // 区分机器人所在区域
    const static uint8_t Area_1 = 0;
    const static uint8_t Area_2 = 1;
    const static uint8_t Area_3 = 2;

    typedef enum
    {
        ORIGIN,            // 刚开机，经过允许操作，将开始自检
        SELF_CHECK,         // 
        READY,              // 自检完成，且无问题

        WORKING,            // 机器人正在正常工作

        STOP,               // 机器人停止工作
    }SysStatus;
}

namespace App
{
    enum Status : uint8_t
    {
        Normal = 0,     // 运行平稳
        Warning = 1,    // 局部异常，正在尝试本地恢复或降级运行
        Error = 2       // 致命异常，相关依赖项需立即响应并保护
    };
}

struct SpiSamp
{
    const char* name = "SpiSamp";               // 采样器名称，仅用于日志定位
    void* owner = nullptr;                      // 采样器所属对象
    bool (*poll_full_frame)(void* owner) = nullptr;
    bool (*consume_full_frame)(void* owner) = nullptr; // 可选的消费回调：读线程产出样本后，由消费线程尽快处理
};

class Application
{
    friend class SystemType;            // 允许系统类访问私有成员
private:
    uint8_t _prescaler_cnt = 0;        // 预分频计数器
    uint8_t prescaler = 1;                  // 应用预分频

    // [状态量] 用于向系统与其他 App 暴露当前健康状况，外部仅具有只读权限
    // 注意：App 内部不要直接修改这个变量，应当覆写下方的 GetStatus()
    App::Status status = App::Normal;

    bool CntFull();                         // 预分频计数器满了没
    
public:
    char name[24];                          // 应用名称

protected:
    // [开机自检] 系统 SELF_CHECK 阶段集中调用，子类按需重写
    // 返回 true 表示硬件/连接正常，自检通过
    virtual bool WatchPoint() { return true; }

    // [状态更新接口] 子类只需覆写此方法描述自身状态
    // 该方法会在 Update 执行前被 System 自动调用并缓存至 status 变量
    virtual App::Status GetStatus() { return App::Normal; }

    Application(const char *name){
        strncpy(this->name, name, 23);
        this->name[23] = 0; // 确保字符串结尾
    };
    virtual void Start() = 0;
    virtual void Update() = 0;
    virtual const std::type_info& GetType() = 0;
};
 
/**
 * @brief 定位模块
 * @note 机器人系统将利用本模块，综合各个信息来源的数据，给出机器人的位置
 */
class Positioner
{
    public:
    Positioner(){};
};


/**
 * @brief 机器人系统
 * @warning 本类也属于：单例
 */
class SystemType
{   
    friend void RobotSystemCpp();
    friend void ApplicationCpp();
    friend void StateCoreCpp();
    friend void SpiReadCpp();
    friend void SpiConsumeCpp();

    SINGLETON(SystemType):Display(*this){};
    

    private:
    void _LedBandControl();
    void _LedBandDisplayControl();

    void _Update_LedBand();
    void _Update_Applications();
    void _Update_SelfCheck(); 
    void _Update_SpiSamps();
    void _Update_SpiConsumes();

    class _LedDisplayAPI
    {
        friend class SystemType;
        private:
        SystemType& entity;
        bool display_overlay = false;    // 是否启用覆盖显示

        typedef enum
        {
            SysLEDDisp_None,
            SysLEDDisp_WarningBlink,
            SysLEDDisp_ErrorBlink,
        }SysLEDDispType;

        uint8_t display_type = 0;           // 显示类型
        

        uint8_t blink_times = 0;            // 闪烁次数
        uint16_t blink_interval = 0;        // 闪烁间隔，单位ms
        uint32_t blink_cnt = 0;             // 闪烁计数器

        public:
        _LedDisplayAPI(SystemType& sys_entity) : entity(sys_entity) {};

        /**
         * @brief 警告闪烁LED
         * @param times 闪烁次数
         * @param interval 闪烁间隔，单位ms，默认400ms
         */
        void WarningBlink(uint8_t times, uint16_t interval = 300);

        /**
         * @brief 错误快闪LED
         * @param times 闪烁次数
         * @param interval 闪烁间隔，单位ms，默认200ms
         */
        void ErrorBlink(uint8_t times, uint16_t interval = 200);

    }Display;
    

    

    /// @brief 机器人当前系统状态
    Systems::SysStatus status = Systems::ORIGIN;           
    /// @brief 机器人所在阵营
    uint8_t camp = Systems::Camp_Blue;

    
    LedWs2812*              led_band = nullptr;             // 仅指 "系统灯"
    Odometer_Ops9           odometer;             // 物理里程计
    Positioner              posner;

    bool is_retrying = false;                               // 是否处于重试状态（从RetryZone出发）
    Application* app_list[24];                              // 系统中的应用实例列表
    SpiSamp* spi_sampler_list[24];                       // SPI采样实例列表


public:
    bool start_selfcheck_flag = false;               // 开始自检标志位
    bool system_ready_flag = false;                  // 系统准备就绪标志位
    bool system_start_to_work_flag = false;         // 系统开始工作标志位

    bool out_from_debugmode = false;

    /// @brief 机器人全局位置，单位m，场地坐标系
    Vec3    position;
    Vec3*   pos_source;                     // 位置来源引用（外部提供）
    void SetPositionSource(Vec3& pos);                     // 设置位置来源

    float runtime_tick;                    // 全局时间戳，单位s

    /// @brief 全局唯一的监控核心
    Monitor& monit = Monitor::GetInstance();
    /// @brief 全局唯一的自动状态机核心
    const StateCore& core = StateCore::GetInstance();
    

    void Init(bool Sc = true);
    
    /// @brief 查找应用实例
    template<typename T>
    T* FindApp(const char* name);
    
    /// @brief 应用注册 
    bool RegistApp(Application& app_inst);
    /// @brief SPI采样任务注册（只注册回调，不处理调度）
    bool RegistSpiSamp(SpiSamp& sampler);

    /// @brief 运行机器人系统主进程
    void Run();

    /// @brief 高性能运行进程（1000Hz）
    void PerformanceRun();
    
    
    private:
    uint16_t ledband_prescaler = 4;                     // 主灯带更新预分频（200 / 4 = 50Hz）
    
};



/* 默认的机器人系统 */
extern SystemType& System;
