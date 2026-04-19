#pragma once
#include "bsp_tim_pwm.hpp"

class Servo
{
public:
    Servo() : angle(0.0f), initialized(false), enabled(false) {};
    
    /// @brief 初始化舵机
    /// @param id 框架分配的TIM不透明句柄
    /// @param channel 定时器通道
    void Init(BSP::TIM::TimID id, uint32_t channel);
    
    void Enable();              // 开始输出PWM信号
    void Disable();             // 停止输出PWM信号
    void SetAngle(float angle); // 角度制，单位：度DEG

    float GetAngle() const { return angle; }
    
private:
    BSP::TIM::PWM pwm_inst;
    float angle;                // 当前舵机角度，单位：度DEG
    bool initialized;
    bool enabled;
};