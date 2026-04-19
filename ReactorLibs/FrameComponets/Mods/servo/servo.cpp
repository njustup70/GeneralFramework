#include "servo.hpp"
#include <cmath>

void Servo::Init(BSP::TIM::TimID id, uint32_t channel)
{
    pwm_inst.Init(id, channel);
    initialized = true;
}

void Servo::Enable()
{
    if (!initialized) return;

    // 确认一下频率等于50Hz，舵机一般都是这个频率
    if (std::fabs(pwm_inst.GetFreq() - 50.0f) > 0.1f) return;
    
    enabled = true;
    pwm_inst.Enable();
}

void Servo::Disable()
{
    if (!initialized) return;
    
    enabled = false;
    pwm_inst.Disable();
}

void Servo::SetAngle(float ang)
{
    if (!initialized) return;
    
    // 假设舵机的工作范围是-90到90度，对应的PWM占空比是2.5%到12.5%
    if (ang < -90.0f) ang = -90.0f;
    if (ang > 90.0f) ang = 90.0f;
    
    // 给类成员赋值
    angle = ang;

    // 通过线性映射计算占空比
    float duty = (angle + 90.0f) / 180.0f * (0.125f - 0.025f) + 0.025f;
    pwm_inst.SetDuty(duty);
}