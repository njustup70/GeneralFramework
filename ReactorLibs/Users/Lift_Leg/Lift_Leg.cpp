#include "Lift_Leg.hpp"

void Lift_Leg::Start()
{
    _enabled = false;

    // 初始化各个电机
    motor_back.Init(Hardware::hcan_main, 5, DJI_C620);
    motor_front_left.Init(Hardware::hcan_main, 6, DJI_C620);
    motor_front_right.Init(Hardware::hcan_main, 7, DJI_C620);
    

    // ADRC 简易初始化（统一参数，后续按实际机构再调）
    motor_front_left.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.2e-4f, 0.30f, 0.005f)
        .ADRC_Limit(18.0f)
        .SpdLimit(3600.0f)
        .ADRC_MaxPlannedVel(3600.0f)
        .ADRC_SOTF(0.33f)
        .Apply();

    motor_front_right.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.2e-4f, 0.30f, 0.005f)
        .ADRC_Limit(18.0f)
        .SpdLimit(3600.0f)
        .ADRC_MaxPlannedVel(3600.0f)
        .ADRC_SOTF(0.33f)
        .Apply();

    motor_back.ConfigADRC()
        .AsPosC()
        .ADRC_Womega(42.0f, 9.6f)
        .ADRC_Physic(4.2e-4f, 0.30f, 0.005f)
        .ADRC_Limit(18.0f)
        .SpdLimit(3600.0f)
        .ADRC_MaxPlannedVel(3600.0f)
        .ADRC_SOTF(0.33f)
        .Apply();

    motor_front_left.driver.Enable();
    motor_front_right.driver.Enable();
    motor_back.driver.Enable();

}

int half_front_stroke = LiftLegConst::FrontStrokeCode / 2;

void Lift_Leg::Update()
{

    if (!_enabled)
    {
        return;
    }

    if (_front_targ_pos_code > half_front_stroke) _front_targ_pos_code = half_front_stroke;
    if (_front_targ_pos_code < -half_front_stroke) _front_targ_pos_code = -half_front_stroke;

    // 左腿向上为正，右腿向上为负
    motor_front_left.SetPos(_front_left_bias + _front_targ_pos_code);
    motor_front_right.SetPos(_front_right_bias -_front_targ_pos_code);

    // 后腿向上为正
    if (_back_targ_pos_code < -LiftLegConst::BackStrokeCode) _back_targ_pos_code = -LiftLegConst::BackStrokeCode;
    if (_back_targ_pos_code > 0) _back_targ_pos_code = 0;

    motor_back.SetPos(_back_bias + _back_targ_pos_code);
}

void Lift_Leg::Enable()
{
    _enabled = true;
}

void Lift_Leg::Disable()
{
    _enabled = false;
}

void Lift_Leg::From_Bottom()
{
    _front_left_bias = half_front_stroke;
    _front_right_bias = -half_front_stroke;
    _back_bias = LiftLegConst::BackStrokeCode;
}

void Lift_Leg::Front_LiftAt400()
{
    _front_targ_pos_code = half_front_stroke - 10000;
}

void Lift_Leg::Front_LiftAt200()
{
    _front_targ_pos_code = half_front_stroke / 2 - 10000;
}

void Lift_Leg::Front_PutDownAt400()
{
    _front_targ_pos_code = -half_front_stroke + 10000;
}

void Lift_Leg::Front_PutDownAt200()
{
    _front_targ_pos_code = -half_front_stroke / 2 + 10000;
}

void Lift_Leg::Front_Origin()
{
    _front_targ_pos_code = 0;
}

void Lift_Leg::Back_PutDownAt200()
{
    _back_targ_pos_code = -LiftLegConst::BackStrokeCode / 2 + 10000;
}

void Lift_Leg::Back_PutDownAt400()
{
    _back_targ_pos_code = -LiftLegConst::BackStrokeCode + 10000;
}

void Lift_Leg::Back_Origin()
{
    _back_targ_pos_code = 0;
}

