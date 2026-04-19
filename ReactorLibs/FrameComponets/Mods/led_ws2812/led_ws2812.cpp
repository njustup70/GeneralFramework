#include "led_ws2812.hpp"
#include "bsp_halport.hpp" // 获取真实 HAL 定义和句柄转换
#include <cmath>           // 替代 arm_math.h
#include "bsp_dwt.hpp"
#include "bsp_log.hpp"

static LedWs2812* targ_led = nullptr;

void LedWs2812::Init(BSP::TIM::TimID id, uint32_t Channel, uint8_t LedNums)
{
    if (id == nullptr)
    {
        BspLog_LogError("[WS2812]: Init at null timer!\r\n");
        return;
    }

    // 记录所用的PWM通道，灯数
    this->id = id;
    this->Channel = Channel;
    this->LedNums = LedNums;

    for(int i = 0; i < MaxLedNums ; i++)
    {
        SetColor(i, 0, 0, 0);
    }

#ifdef USE_REAL_HAL
    // 【修改这里】：直接用 reinterpret_cast 强转，绕过烦人的底层 struct 声明限制
    TIM_HandleTypeDef* htim = reinterpret_cast<TIM_HandleTypeDef*>(this->id);
    PwmMaxValue = __HAL_TIM_GET_AUTORELOAD(htim);
#else
    // 虚拟环境下的假值
    PwmMaxValue = 100;
#endif

    HIGH_WS2812 = PwmMaxValue * 0.67f;    // PWM高电平数值
    LOW_WS2812  = PwmMaxValue * 0.33f;    // PWM低电平数值   

    // 注册全局指针以供中断回调使用
    targ_led = this;
}

/**
 * @brief 设置某个LED的颜色
 * @warning 输入的RGB为0~255范围
 */
void LedWs2812::SetColor(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B)
{
    // 不允许 Led_id 大于 实际拥有的灯数
    while(Led_id > (LedNums - 1))   Led_id -= LedNums;
    while(Led_id < 0)   Led_id += LedNums;

    // 向 数组 覆写颜色
    int i = 0;
    for(i=0; i<8; i++)  WS2812buf2send[Led_id][i]    = ( G & (1 << (7 -i))? (HIGH_WS2812):LOW_WS2812 ); 
    for(i=8; i<16; i++) WS2812buf2send[Led_id][i]   = ( R & (1 << (15-i))? (HIGH_WS2812):LOW_WS2812 ); 
    for(i=16; i<24; i++) WS2812buf2send[Led_id][i]  = ( B & (1 << (23-i))? (HIGH_WS2812):LOW_WS2812 ); 
}

/**
 * @brief 两个颜色的流动渐变模式
 */
void LedWs2812::GradientFlow(Color color_0, Color color_1, float period)
{
    color_0 = color_0 * BiasFactor;
    color_1 = color_1 * BiasFactor;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        RuntimeCnt = std::fmod(RuntimeCnt, period);
    }

    float phaseStep = (float)period / LedNums;
    
    for (int i = 0; i < LedNums; i++)
    {
        float phase = std::fmod(RuntimeCnt + (i * phaseStep), period);
        float ratio = 0.5f * (1.0f + std::sin(2 * 3.1415 * phase / period - 1.5708));
        Color color_targ = (color_0 + (color_1 - color_0) * ratio);
        SetColor(i, color_targ.r, color_targ.g, color_targ.b);
    }
}

/**
 * @brief 单色呼吸灯
 */
void LedWs2812::Breath(Color color_0, float period)
{
    color_0 = color_0 * BiasFactor;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        RuntimeCnt = std::fmod(RuntimeCnt, period);
    }

    float ratio = 0.5f * (1.0f + std::sin((2 * 3.1415926f) * RuntimeCnt / period - (3.1415926f/2)));
    Color color_targ = color_0 * ratio;
    Fill(color_targ.r, color_targ.g, color_targ.b);
}

/**
 * @brief 单色跑马灯
 */
void LedWs2812::Running(Color color, float width, float period)
{
    color = color * BiasFactor;

    float delta_t = DWT_GetDeltaTime(&dwt_tick);
    RuntimeCnt += delta_t;
    if (RuntimeCnt >= period) {
        RuntimeCnt = std::fmod(RuntimeCnt, period);
    }

    float position = (RuntimeCnt / period) * LedNums;
    int width_in_leds = width * LedNums;

    for (int i = 0; i < LedNums; i++)
    {
        float distance = std::fabs(i - position);
        if (distance <= (width_in_leds / 2) || distance >= (LedNums - width_in_leds / 2)) {
            SetColor(i, color.r, color.g, color.b);
        } else {
            SetColor(i, 0, 0, 0);
        }
    }
}

void LedWs2812::Lit(Color color)
{
    RuntimeCnt += DWT_GetDeltaTime(&dwt_tick);
    if (RuntimeCnt > 0.5)
    {
        color = color * BiasFactor;
        Fill(color.r, color.g, color.b);
        RuntimeCnt = 0;
    }
}

void LedWs2812::Expand(Color color_0, float period)
{
    float delta_t = DWT_GetDeltaTime(&dwt_tick);

    if (RuntimeCnt < period)
    {
        RuntimeCnt += delta_t;
        float ratio = RuntimeCnt / period;
        float center = LedNums / 2.0f;
        
        for (int i = 0; i < LedNums; i++)
        {
            float litness = 0;
            if (i < center)     litness = (i / center) + 2 * ratio - 1;
            else                litness = 2 * ratio - ((float)i - center) / center;
            
            if (litness < 0) litness = 0;
            if (litness > 1) litness = 1;
            Color color_targ = color_0 * litness;
            SetColor(i, color_targ.r, color_targ.g, color_targ.b);
        }
    }
}

void LedWs2812::Fill(uint8_t R, uint8_t G, uint8_t B)
{
    for(int i = 0; i < LedNums ; i++)
    {
        SetColor(i, R, G, B);
    }
}

/**
 * @brief 发送数据，更新LED的颜色（使用PWM + DMA）
 */
void LedWs2812::Upload()
{
    if (this->id == nullptr) return;

#ifdef USE_REAL_HAL
    // 【修改这里】：解包指针
    TIM_HandleTypeDef* htim = reinterpret_cast<TIM_HandleTypeDef*>(this->id);
    
    // WS2812 时序要求：提前拉高 CNT 使其产生更新事件
    htim->Instance->CNT = htim->Instance->ARR - 1;
    HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)WS2812buf2send, LedNums * 24 + 1);
#endif
}

#ifdef USE_REAL_HAL
/**
 * @brief 覆写PWM + DMA发送完成中断
 * @note 必须加上 extern "C" 才能正确覆盖 HAL 库里面的弱函数 (Weak function)
 */
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // 【修改这里】：解包指针做对比
    if(targ_led != nullptr && htim == reinterpret_cast<TIM_HandleTypeDef*>(targ_led->id))
    {
        HAL_TIM_PWM_Stop_DMA(htim, targ_led->Channel);
    }
}
#endif