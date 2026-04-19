#include "bsp_tim_pwm.hpp"
#include "bsp_halport.hpp" 

using namespace BSP::TIM;

// ---- PWM 类方法实现 ----

void PWM::UpdateFreq()
{
    if (!id_) return;

#ifdef USE_REAL_HAL
    // 【核心修复】：直接用 reinterpret_cast 强转解包，告别类型匹配报错
    TIM_HandleTypeDef* htim = reinterpret_cast<TIM_HandleTypeDef*>(id_);
    
    // 计算PWM频率 = 定时器时钟频率 / (ARR + 1)
    uint32_t timer_clock_freq = HAL_RCC_GetPCLK1Freq() * 2 / (htim->Instance->PSC + 1);   
    freq_ = (float)timer_clock_freq / (arr_ + 1);
#else
    freq_ = 50.0f; // 虚拟环境默认给50Hz防除零
#endif
}

void PWM::Init(TimID id, uint32_t channel)
{
    id_ = id;
    channel_ = channel;

    if (!id_) return;

#ifdef USE_REAL_HAL
    // 【核心修复】：直接解包
    TIM_HandleTypeDef* htim = reinterpret_cast<TIM_HandleTypeDef*>(id_);
    arr_ = __HAL_TIM_GET_AUTORELOAD(htim);
    ccr_ = __HAL_TIM_GET_COMPARE(htim, channel_);
#else
    arr_ = 19999;
    ccr_ = 0;
#endif

    UpdateFreq();
    SetDuty(0.0f); // 默认初始占空比为 0
}

void PWM::SetDuty(float duty)
{
    if (!id_) return;

    // 限制范围在 0.0 ~ 1.0 之间
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    duty_ = duty;
    ccr_ = (uint32_t)(arr_ * duty);

#ifdef USE_REAL_HAL
    // 【核心修复】：直接解包
    TIM_HandleTypeDef* htim = reinterpret_cast<TIM_HandleTypeDef*>(id_);
    __HAL_TIM_SET_COMPARE(htim, channel_, ccr_);
#endif
}

void PWM::Enable()
{
    if (!id_) return;

#ifdef USE_REAL_HAL
    TIM_HandleTypeDef* htim = reinterpret_cast<TIM_HandleTypeDef*>(id_);
    HAL_TIM_PWM_Start(htim, channel_);
#endif
    enabled_ = true;
}

void PWM::Disable()
{
    if (!id_) return;

#ifdef USE_REAL_HAL
    TIM_HandleTypeDef* htim = reinterpret_cast<TIM_HandleTypeDef*>(id_);
    HAL_TIM_PWM_Stop(htim, channel_);
#endif
    enabled_ = false;
}