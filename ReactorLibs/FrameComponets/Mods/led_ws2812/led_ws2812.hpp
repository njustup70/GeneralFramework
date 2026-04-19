/**
 * @author Huangney
 * @note 适配框架重构版：解耦HAL库，使用 TimID 句柄标识
 */
#ifndef _LED_WS2812_HPP_
#define _LED_WS2812_HPP_ 
#include "std_math.hpp"
#include "bsp_tim_pwm.hpp"

class LedWs2812
{
private:
    const static int MaxLedNums = 16;

    uint32_t PwmMaxValue;       // PWM最大值
    uint32_t HIGH_WS2812 = 0;   // PWM高电平数值
    uint32_t LOW_WS2812 = 0;    // PWM低电平数值
    uint32_t dwt_tick;

    float RuntimeCnt = 0;
    uint32_t WS2812buf2send[MaxLedNums][24] = {0};
    
public:
    LedWs2812(){};
    ~LedWs2812(){};
    
    // 使用不透明指针替代 TIM_HandleTypeDef*
    BSP::TIM::TimID id;         
    uint32_t Channel;
    uint8_t LedNums;            // LED灯珠数量

    Vec3 BiasFactor;            // 颜色偏置因子（用于校正颜色）
    
    void SetColor(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B);
    void Upload();
    
    // 初始化接口改为接受 TimID
    void Init(BSP::TIM::TimID id, uint32_t Channel, uint8_t LedNums);
    void Fill(uint8_t R, uint8_t G, uint8_t B);

    void Breath(Color color_0, float period);
    void GradientFlow(Color color_0, Color color_1, float period);
    void Running(Color color, float width, float period);
    void Lit(Color color);     // 供外部调用的Fill，省算力
    void Expand(Color color_0, float period);
};

#endif