#pragma once
#include <cstdint>

namespace BSP
{
    namespace TIM
    {
        // 不透明指针 TIM_HandleTypeDef* 的框架解耦替代
        struct OpaqueTim;
        using TimID = OpaqueTim*;

        /**
         * @brief PWM 实例类
         */
        class PWM
        {
        public:
            PWM() : id_(nullptr), channel_(0), freq_(0.0f), enabled_(false), arr_(0), ccr_(0), duty_(0.0f) {}

            /// @brief 初始化PWM实例
            /// @param id 目标硬件定时器的不透明句柄
            /// @param channel PWM通道 (如 TIM_CHANNEL_1)
            void Init(TimID id, uint32_t channel);

            /// @brief 设置占空比
            /// @param duty 占空比，范围 [0.0f, 1.0f]
            void SetDuty(float duty);

            /// @brief 使能PWM输出
            void Enable();

            /// @brief 关闭PWM输出
            void Disable();

            /// @brief 获取当前计算得出的PWM频率
            float GetFreq() const { return freq_; }

            /// @brief 获取当前占空比
            float GetDuty() const { return duty_; }

        private:
            TimID id_;
            uint32_t channel_;
            float freq_;
            bool enabled_;

            uint32_t arr_;
            uint32_t ccr_;
            float duty_;

            /// @brief 内部刷新频率
            void UpdateFreq();
        };
    }
}