#pragma once
#include "System.hpp"
#include "motor_dm.hpp"
#include "motor_dji.hpp"
#include "bsp_gpio.hpp"
#include "servo.hpp"
// 块中心在场地坐标系的位置，xy 单位为米，height 单位为毫米
struct BlockInfo
{
    float x;
    float y;
    float height;
};

// 12 个物块的标准坐标（红方）
// TODO: 梅林位置不固定，后续优化遍历方式
const BlockInfo RED_ALL_BLOCKS_DATA[12] =
{
    {1.8f+2*1.2f, 3.8f+3*1.2f, 200.0f}, {1.8f+1*1.2f, 3.8f+3*1.2f, 400.0f}, {1.8f+0*1.2f, 3.8f+3*1.2f, 200.0f}, // 1-3
    {1.8f+2*1.2f, 3.8f+2*1.2f, 400.0f}, {1.8f+1*1.2f, 3.8f+2*1.2f, 600.0f}, {1.8f+0*1.2f, 3.8f+2*1.2f, 400.0f}, // 4-6
    {1.8f+2*1.2f, 3.8f+1*1.2f, 600.0f}, {1.8f+1*1.2f, 3.8f+1*1.2f, 400.0f}, {1.8f+0*1.2f, 3.8f+1*1.2f, 200.0f}, // 7-9
    {1.8f+2*1.2f, 3.8f+0*1.2f, 400.0f}, {1.8f+1*1.2f, 3.8f+0*1.2f, 200.0f}, {1.8f+0*1.2f, 3.8f+0*1.2f, 400.0f}  // 10-12
};

class GetBlock : public Application
{
    SINGLETON(GetBlock):Application("GetBlock"){};
    APPLICATION_OVERRIDE

public:
    MotorDM  rolldmmotor; // 翻滚电机（达妙）
    
    MotorDJI suckmotor[2];    // 摩擦带吸吮电机（大疆 M2006，CAN2 ID:左1，右2）（顺时针）
    MotorDJI stretchmotor[2];     // 伸缩电机（大疆 M2006，CAN2 ID:左4，右3）
    MotorDJI liftmotor[2];// 抬升电机（大疆 M3508，CAN1 ID:左5，右6）

Servo liftservo[2]; // 抬升舵机（大疆 M3508，CAN1 ID:左5，右6）预留

 BSP::GPIO::Inst air_pump_pin;

private:
    bool enabled = false;

    enum BlockState
    {
        STATE_IDLE = 0,
        STATE_INIT,
        STATE_LIFTED,
        STATE_GET200BLOCK,
        STATE_GET400BLOCK,
        STATE_GET600BLOCK,
        STATE_RELEASEBLOCK,
        STATE_EMERGENCY
    };
    BlockState appstate = STATE_IDLE;

public:
    // 数组顺序：rolldmmotor->0, stretchmotor->1, suckmotor->2
    float target_state_pos[7]   = {0.0f}; // 七个电机的目标位置
    float target_state_speed[7] = {0.0f}; // 七个电机的目标速度

    // 软限位：[电机][0]=min, [1]=max
    // rolldmmotor 单位 rad，suckmotor/stretchmotor/liftmotor 单位 code（total_angle）
    float pos_limit[7][2] = {{0.0f, 0.0f},
                              {0.0f, 0.0f},
                              {0.0f, 0.0f},
                              {0.0f, 0.0f},
                              {0.0f, 0.0f},
                              {0.0f, 0.0f}
                              ,{0.0f, 0.0f}};

    // 取 200/400/600 块时抬升电机3508对应的 total_angle 目标值
    float blockheight_2_liftmotortargetpos[3] = {0.0f, 520000.0f, 960000.0f};

    // 遥控器按键边沿检测
    uint8_t last_btn_state[8] = {0};
    bool    btn_enter[8]      = {false};

    // 当前帧目标 KFS 信息
    BlockInfo target_block_pos[3] = {{0}};
    uint8_t   target_count        = 0;

    void Enable();
    void Stop();

    /**
     * @brief 设置三个电机的目标状态并立即下发
     * @param strech_pos   伸出电机目标位置（code，total_angle 语义）
     * @param suck_pos      吸吮电机目标位置（code，total_angle 语义）
     * @param lift_pos       抬升目标位置（code，total_angle 语义）

     * @param strech_speed 伸出电机速度（大疆位置串级模式，此参数保留但当前忽略）
     * @param suck_speed    吸吮电机速度（同上）
     * @param lift_speed     抬升速度（同上）
     */
    void SetTargetState(float stretch_pos_L = 0.0f, float stretch_pos_R = 0.0f,
                        float suck_pos_L = 0.0f,    float suck_pos_R = 0.0f,
                        float lift_pos_L = 0.0f,    float lift_pos_R = 0.0f,
                        float stretch_speed_L = 2.0f, float stretch_speed_R = 2.0f,
                        float suck_speed_L = 2.0f,    float suck_speed_R = 2.0f,
                        float lift_speed_L = 2.0f,    float lift_speed_R = 2.0f);

    void SetPosLimit(float stretch_min_L, float stretch_max_L, float stretch_min_R, float stretch_max_R,
                     float suck_min_L, float suck_max_L, float suck_min_R, float suck_max_R,
                     float lift_min_L, float lift_max_L, float lift_min_R, float lift_max_R);

    void Get_200Block();
    void Get_400Block();
    void Get_600Block();
    void ReleaseBlock();

    void Action_LiftToHeight(float height); // TODO: 预留

    void GetTargetBlockInfo();
};
