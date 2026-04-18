#pragma once

#include "System.hpp"

namespace LiftLegConst
{
    // 实际值为1034000，但为了安全起见，暂时限制在更小的范围内
    const int FrontStrokeCode = 1020000; // 前腿行程（单位：编码值）
    
    const int BackStrokeCode = 650000;  // 后腿行程（单位：编码值）
}

class Lift_Leg : public Application
{
    SINGLETON(Lift_Leg) : Application("Lift_Leg") {}
    APPLICATION_OVERRIDE

public:
    MotorDJI motor_front_left;
    MotorDJI motor_front_right;

    // 后腿是二合一的
    MotorDJI motor_back;

    void Enable();
    void Disable();

    /// @note 如果是在调车架上，那么四条腿是从底部启动的。
    void From_Bottom();

    void Front_LiftAt400();
    void Front_LiftAt200();
    void Front_PutDownAt400();
    void Front_PutDownAt200();
    void Front_Origin();

    void Back_PutDownAt200();
    void Back_PutDownAt400();
    void Back_Origin();

    bool _enabled = false;

private:
    /// @brief 前腿的目标位置
    int _front_targ_pos_code = 0;
    /// @brief 后腿的目标位置
    int _back_targ_pos_code = 0;

    int _front_left_bias = 0;
    int _front_right_bias = 0;
    int _back_bias = 0;
};
