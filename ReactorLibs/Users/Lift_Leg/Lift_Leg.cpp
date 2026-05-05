#include "Lift_Leg.hpp"
#include "InterBoardComm.hpp"

Lift_Leg &lift_leg = Lift_Leg::GetInstance();

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
    
    Pin little_yellow_pin[3] = {{.port='E', .number=9}, {.port='E', .number=11}, {.port='E', .number=13}};
    for(int i = 0; i < 3; i++)
    {
        little_yellow[i].Init(little_yellow_pin[i]);
    }

    chassis_board.RegisterTask(2, LiftLegCmdRxCallback, this);
}

int half_front_stroke = LiftLegConst::FrontStrokeCode / 2;

void Lift_Leg::Update()
{

    if (!_enabled)
    {
        return;
    }

    if(System.out_from_debugmode)
    {
        motor_front_left.Neutral();
        motor_front_right.Neutral();
        motor_back.Neutral();
        return;
    }

    if (_front_targ_pos_code > half_front_stroke) _front_targ_pos_code = half_front_stroke;
    if (_front_targ_pos_code < -half_front_stroke) _front_targ_pos_code = -half_front_stroke;

    // 左腿向上为正，右腿向上为负
    motor_front_left.SetPos(_front_left_start_bias + _front_left_bias + _front_targ_pos_code);
    motor_front_right.SetPos(_front_right_start_bias + _front_right_bias - _front_targ_pos_code);

    // 后腿向上为正
    if (_back_targ_pos_code < -LiftLegConst::BackStrokeCode) _back_targ_pos_code = -LiftLegConst::BackStrokeCode;
    if (_back_targ_pos_code > 0) _back_targ_pos_code = 0;

    motor_back.SetPos(_back_bias + _back_targ_pos_code);
    // if(little_yellow[0].Read())
    // {
    //     return;
    // }
    // if(little_yellow[1].Read())
    // {
    //     return;
    // }
    // if(little_yellow[2].Read())
    // {        
    //     return;
    // }
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

void Lift_Leg::LiftLegCmdRxCallback(uint8_t task_id, const uint8_t* payload, uint8_t payload_len, void* user_ctx)
{
    if(payload == nullptr || payload_len == 0)
    {
        return;
    }

    uint8_t cmd = payload[0];
    
    switch(cmd)
    {
        // 前腿抬高200
        case 0x02:
            GetInstance().Front_LiftAt200();
        break;

        // 前腿放低200
        case 0x06:
            GetInstance().Front_PutDownAt200();
        break;

        // 前腿抬高400
        case 0x03:
            GetInstance().Front_LiftAt400();
        break;

        // 前腿放低400
        case 0x07:
            GetInstance().Front_PutDownAt400();
        break;

        // 前腿归位
        case 0x04:
            GetInstance().Front_Origin();
        break;

        // 后腿放低200
        case 0x0B:
            GetInstance().Back_PutDownAt200();
        break;

        // 后腿放低400
        case 0x0F:
            GetInstance().Back_PutDownAt400();
        break;  
        // 后腿归位
        case 0x0C:
            GetInstance().Back_Origin();        
        break;          
    }
}