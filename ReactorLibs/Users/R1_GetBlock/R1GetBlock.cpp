#include "R1GetBlock.hpp"
#include <algorithm>
#include <cmath>
#include "farcon.hpp"
#include "bsp_hardware.hpp"

extern Farcon farcon;
extern SystemType &System;

// 遥控器按键 8 存储高度的计数器（1→存200块高度，2→400，3→600）
static uint8_t entertime = 0;
int suck_flag = 0;
float lift_target_pos = 0.0f;

float lift_pos_kp = 2.0f;
float lift_pos_ki = 0.0f;
float lift_pos_kd = 0.0f;

float lift_speed_kp = 0.001f;
float lift_speed_ki = 0.0f;
float lift_speed_kd = 0.0f;

int air_flag = 0;

float debug_angle=0;
void GetBlock::Start()
{
  air_pump_pin = BSP::GPIO::Inst({'D', 12});
  // ---- 达妙翻滚电机 ----
  // rolldmmotor.Init(Hardware::hcan_sub, 0x11, 0x10, DM_MODE_POSANDVEL);
  // rolldmmotor.SetAutoEnable(1, 500);
  // rolldmmotor.Enable();

  // ---- 大疆抬升电机左（M3508，减速比19，CAN1 ID:5，位置串级模式）----
  liftmotor[0].Init(Hardware::hcan_main, 5, DJI_C620);
  liftmotor[0].ConfigPID().AsPosC().Pos_Coeff(1.5f, 0.0f, 0.3f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(500.0f, 4000.0f)                               // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.04f, 0.003, 0.0f)                            // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(3.0f, 10.0f)                                   // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  liftmotor[0].driver.Enable(); // 设负的向上，正的向下

  // ---- 大疆抬升电机右（M3508，减速比19，CAN1 ID:6，位置串级模式）----
  liftmotor[1].Init(Hardware::hcan_main, 6, DJI_C620);
  liftmotor[1].ConfigPID().AsPosC().Pos_Coeff(1.8f, 0.0f, 0.3f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(500.0f, 4000.0f)                               // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.06f, 0.004, 0.0f)                            // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(3.0f, 10.0f)                                   // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  liftmotor[1].driver.Enable(); // 设正的向上，负的向下

  // ---- 大疆吸吮电机左（M2006，减速比36，CAN2 ID:1，位置串级模式）----
  suckmotor[0].Init(Hardware::hcan_sub, 1, DJI_C610);
  suckmotor[0].ConfigPID().AsPosC().Pos_Coeff(20.0f, 0.0f, 2.5f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(300.0f, 5000.0f)                                // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.003f, 0.00005f, 0.0f)                         // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(2.0f, 10.0f)                                    // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  suckmotor[0].driver.Enable(); // 左边target_pos是1000000左右合适，且－的往外推，正的往里吸

  // ---- 大疆吸吮电机右（M2006，减速比36，CAN2 ID:2，位置串级模式）----
  suckmotor[1].Init(Hardware::hcan_sub, 2, DJI_C610);
  suckmotor[1].ConfigPID().AsPosC().Pos_Coeff(20.0f, 0.0f, 2.5f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(300.0f, 5000.0f)                                // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.003f, 0.00005f, 0.0f)                         // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(2.0f, 10.0f)                                    // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  suckmotor[1].driver.Enable(); // 右边target_pos是1000000左右合适，且－的往外推，正的往里吸

  //   // ---- 大疆伸缩电机左（M2006，减速比36，CAN2 ID:4，位置串级模式）----
  stretchmotor[0].Init(Hardware::hcan_sub, 4, DJI_C610);
  stretchmotor[0].ConfigPID().AsPosC().Pos_Coeff(15.0f, 0.0f, 1.0f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(300.0f, 4000.0f)                                   // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.01f, 0.00005f, 0.0f)                             // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(2.0f, 10.0f)                                       // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  stretchmotor[0].driver.Enable(); // 左边target_pos是1000000左右合适，且+的往前

  // ---- 大疆伸缩电机右（M2006，减速比36，CAN2 ID:3，位置串级模式）----
  stretchmotor[1].Init(Hardware::hcan_sub, 3, DJI_C610);
  stretchmotor[1].ConfigPID().AsPosC().Pos_Coeff(15.0f, 0.0f, 1.0f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(300.0f, 4000.0f)                                   // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.01f, 0.00005f, 0.0f)                             // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(2.0f, 10.0f)                                       // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  stretchmotor[1].driver.Enable(); // 右边target_pos是1000000左右合适，且-的往前

  SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  liftservo[0].Init(ToID(&htim5), TIM_CHANNEL_2);//-35度锁死，0度松开
  liftservo[0].Enable();

  liftservo[1].Init(ToID(&htim5), TIM_CHANNEL_3);//15度锁死，0度松开
  liftservo[1].Enable();

  appstate = STATE_INIT;
  Enable();
}

// ======================== Update ========================

// void GetBlock::Update()
// {
//   //   GetTargetBlockInfo();

//     // ---- 状态机 ----
//     if (appstate == STATE_INIT)
//     {
//       rolldmmotor.SetPosVel(0.0f, 2.0f);
//       // 参数顺序：左伸缩, 右伸缩, 左吸吮, 右吸吮, 左抬升, 右抬升
//       SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//       release_air_pin.Write(true);
//     }

//     if (appstate == STATE_LIFTED)
//     {
//       // 修复：必须明确指定读取数组中的某一个电机（比如左伸缩电机[0]）的反馈角度
//       if (stretchmotor[0].driver.measure.total_angle < -700000.0f)
//       {
//         rolldmmotor.SetPosVel(-2.30383492f, 2.0f);
//         // 原逻辑：lift(抬升)运动到 -750000，伸缩回零。吸吮保持原样([5], [6])
//         SetTargetState(0.0f, 0.0f,
//                        target_state_pos[5], target_state_pos[6],
//                        -750000.0f, -750000.0f);
//       }
//       appstate = STATE_IDLE;
//     }

//     if (appstate == STATE_IDLE)
//     {
//       release_air_pin.Write(true);
//     }

//     // ---- 遥控器按键边沿检测 ----
//     for (int i = 0; i < 8; i++)
//     {
//       uint8_t cur = farcon.button_first_half[i];
//       btn_enter[i] = (cur == 1 && last_btn_state[i] == 0); // 上升沿
//       last_btn_state[i] = cur;
//     }

//     // ---- 按键动作 ----

//     // 按键 1/2/3：一级动作，直接调用已经封装好的取块函数，代码更简洁
//     if (farcon.button_first_half[0] == 1)
//     {
//       Get_200Block();
//     }
//     if (farcon.button_first_half[1] == 1)
//     {
//       Get_400Block();
//     }
//     if (farcon.button_first_half[2] == 1)
//     {
//       Get_600Block();
//     }

//     // 按键 4：二级动作，滑台(伸缩)全伸出吸附
//     if (farcon.button_first_half[3] == 1)
//     {
//       rolldmmotor.SetPosVel(-2.30383492f, 2.0f);
//       // 伸缩(Stretch)伸出到 430000，吸吮([5][6])和抬升([1][2])保持当前状态
//       SetTargetState(430000.0f, 430000.0f,
//                      target_state_pos[5], target_state_pos[6],
//                      target_state_pos[1], target_state_pos[2]);
//       vacuum_pump_pin.Write(true);
//     }

//     // 按键 5：放块，调用封装好的释放函数
//     if (farcon.button_first_half[4] == 1)
//     {
//       ReleaseBlock();
//       appstate = STATE_IDLE; // 释放后重置回空闲状态
//     }

//     // 按键 6/7：手动微调抬升高度（单次触发，每次约 5cm）
//     if (farcon.button_first_half[5] == 1 && btn_enter[5])
//     {
//       // 微调时，必须同时更新左右抬升电机的值
//       target_state_pos[1] -= 166666.0f; // 左抬升上升
//       target_state_pos[2] -= 166666.0f; // 右抬升上升

//       SetTargetState(target_state_pos[3], target_state_pos[4],
//                      target_state_pos[5], target_state_pos[6],
//                      target_state_pos[1], target_state_pos[2]);
//     }
//     if (farcon.button_first_half[6] == 1 && btn_enter[6])
//     {
//       target_state_pos[1] += 166666.0f; // 左抬升下降
//       target_state_pos[2] += 166666.0f; // 右抬升下降

//       SetTargetState(target_state_pos[3], target_state_pos[4],
//                      target_state_pos[5], target_state_pos[6],
//                      target_state_pos[1], target_state_pos[2]);
//     }

//     // 按键 8：依次存储 200/400/600 块对应的当前抬升位置
//   //   if (farcon.button_first_half[7] == 1 && btn_enter[7])
//   //   {
//   //     // 取左侧抬升电机([1])的当前目标高度存入字典即可
//   //     blockheight_2_liftmotortargetpos[entertime] = target_state_pos[1];
//   //     entertime = (entertime + 1) % 3;
//   //     // TODO: 将 entertime 回传到遥控器屏幕
//   //   }
// }

void GetBlock::Update()
{

  GetTargetBlockInfo();
  if (System.out_from_debugmode)
  {
    Stop();
  }

  if (air_flag == 1)
  {
    air_pump_pin.Write(true);
  }
  else if (air_flag == 0)
  {
    air_pump_pin.Write(false);
  }

  // ---- 状态机 ----
  if (appstate == STATE_INIT)
  {
    //   rolldmmotor.SetPosVel(0.0f, 2.0f);
    // 参数顺序：左伸缩, 右伸缩, 左吸吮, 右吸吮, 左抬升, 右抬升
    if (suck_flag == 1)
    {
      SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    }
    if (suck_flag == 2)
    {
      SetTargetState(3900000.0f, 0.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    liftservo[0].SetAngle(-35);
     liftservo[1].SetAngle(15);
    }
    if (suck_flag == 3)
    {
      SetTargetState(3900000.0f, 3800000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    }
    if (suck_flag == 4)
    {
      SetTargetState(3900000.0f, 3800000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    }

    if (suck_flag == 5)
    {
      SetTargetState(3900000.0f, 3800000.0f, 20000000.0f, 20000000.0f, lift_target_pos, lift_target_pos);
      for (int i = 0; i < 2000000; i++)
      {
      }
      suck_flag = 6;
    }
    if (suck_flag == 6)
    {
      air_pump_pin.Write(1);
    }
    if (suck_flag == 7)
    {
      air_pump_pin.Write(0);
      SetTargetState(0.0f, 0.0f, 20000000.0f, 20000000.0f, lift_target_pos, lift_target_pos);
    }
    // 回去
    if (suck_flag == 8)
    {
      SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
          liftservo[0].SetAngle(0);
     liftservo[1].SetAngle(0);
      suck_flag = 0;
    }
    //   if(farcon.button_first_half[0] == 1)
    //   {
    //     suck_flag = 1;
    //   }
    //   if(farcon.button_first_half[1] == 1)
    //   {
    //     suck_flag = 2;
    //   }
    // if(farcon.button_first_half[2] == 1)
    //   {
    //     suck_flag = 3;
    //   }
    //   if(farcon.button_first_half[3] == 1)
    //   {
    //     suck_flag = 4;
    //   }
    //   if(farcon.button_first_half[4] == 1)
    //   {
    //     suck_flag = 5;
    //   }

    //   if(farcon.button_second_half[0] == 1)
    //   {
    //     lift_target_pos=0.0f;
    //   }else  if(farcon.button_second_half[1] == 1)
    //   {
    //     lift_target_pos=1800000.0f;
    //   }
    //   else  if(farcon.button_second_half[2] == 1)
    //   {
    //     lift_target_pos=380000.0f;
    //   }
  }
}

// ======================== Enable / Stop ========================

void GetBlock::Enable()
{
  enabled = true;
}

void GetBlock::Stop()
{
  suckmotor[0].Neutral(); // 摩擦带吸吮电机（大疆 M2006，CAN2 ID:左1，右2）（顺时针）
  suckmotor[0].driver.Disable();
  suckmotor[1].Neutral();
  suckmotor[1].driver.Disable();
  stretchmotor[0].Neutral(); // 伸缩电机（大疆 M2006，CAN2 ID:左4，右3）
  stretchmotor[0].driver.Disable();
  stretchmotor[1].Neutral();
  stretchmotor[1].driver.Disable();
  liftmotor[0].Neutral(); // 抬升电机（大疆 M3508，CAN1 ID:左5，右6）
  liftmotor[0].driver.Disable();
  liftmotor[1].Neutral();
  liftmotor[1].driver.Disable();
  air_pump_pin.Write(false);
  liftservo[0].Disable();
  liftservo[1].Disable();
  enabled = false;
}

// ======================== SetTargetState ========================

void GetBlock::SetTargetState(float stretch_pos_L, float stretch_pos_R,
                              float suck_pos_L, float suck_pos_R,
                              float lift_pos_L, float lift_pos_R,
                              float stretch_speed_L, float stretch_speed_R,
                              float suck_speed_L, float suck_speed_R,
                              float lift_speed_L, float lift_speed_R)
{
  // 1->左抬升, 2->右抬升
  target_state_pos[1] = -lift_pos_L;
  target_state_pos[2] = lift_pos_R;
  target_state_speed[1] = lift_speed_L;
  target_state_speed[2] = lift_speed_R;

  // 3->左伸缩, 4->右伸缩
  target_state_pos[3] = stretch_pos_L;
  target_state_pos[4] = -stretch_pos_R;
  target_state_speed[3] = stretch_speed_L;
  target_state_speed[4] = stretch_speed_R;

  // 5->左吸吮, 6->右吸吮
  target_state_pos[5] = -suck_pos_L;
  target_state_pos[6] = suck_pos_R;
  target_state_speed[5] = suck_speed_L;
  target_state_speed[6] = suck_speed_R;

  // 软限位约束
  for (int i = 1; i <= 6; i++)
  {
    //    target_state_pos[i] = std::clamp(target_state_pos[i], pos_limit[i][0], pos_limit[i][1]);
  }

  // 下发给大疆电机
  liftmotor[0].SetPos(target_state_pos[1]);
  liftmotor[1].SetPos(target_state_pos[2]);

  stretchmotor[0].SetPos(target_state_pos[3]);
  stretchmotor[1].SetPos(target_state_pos[4]);

  suckmotor[0].SetPos(target_state_pos[5]);
  suckmotor[1].SetPos(target_state_pos[6]);
}

// ======================== SetPosLimit ========================

void GetBlock::SetPosLimit(float stretch_min_L, float stretch_max_L, float stretch_min_R, float stretch_max_R,
                           float suck_min_L, float suck_max_L, float suck_min_R, float suck_max_R,
                           float lift_min_L, float lift_max_L, float lift_min_R, float lift_max_R)
{
  // 左电机限位 (索引 1, 3, 5)
  pos_limit[1][0] = lift_min_L;
  pos_limit[1][1] = lift_max_L;
  pos_limit[3][0] = stretch_min_L;
  pos_limit[3][1] = stretch_max_L;
  pos_limit[5][0] = suck_min_L;
  pos_limit[5][1] = suck_max_L;

  // 右电机限位 (索引 2, 4, 6)
  pos_limit[2][0] = lift_min_R;
  pos_limit[2][1] = lift_max_R;
  pos_limit[4][0] = stretch_min_R;
  pos_limit[4][1] = stretch_max_R;
  pos_limit[6][0] = suck_min_R;
  pos_limit[6][1] = suck_max_R;
}

// ======================== 取块动作 ========================

void GetBlock::Get_200Block()
{
  appstate = STATE_GET200BLOCK;
  rolldmmotor.SetPosVel(-2.30383492f, 2.0f);

  // 参数顺序：左伸缩, 右伸缩, 左吸吮, 右吸吮, 左抬升, 右抬升
  // 如果右抬升反装需要相反数，就可以直接写 -blockheight_2_liftmotortargetpos[0]
  float target_lift = blockheight_2_liftmotortargetpos[0];
  SetTargetState(130000.0f, 130000.0f, 0.0f, 0.0f, target_lift, target_lift);

  //  vacuum_pump_pin.Write(true);
}
// ======================== 取块/放块动作 ========================

void GetBlock::Get_400Block()
{
  appstate = STATE_GET400BLOCK;
  rolldmmotor.SetPosVel(-2.30383492f, 2.0f);

  float target_lift = blockheight_2_liftmotortargetpos[1];
  // 参数顺序：左伸缩, 右伸缩, 左吸吮, 右吸吮, 左抬升, 右抬升
  SetTargetState(130000.0f, 130000.0f, 0.0f, 0.0f, target_lift, target_lift);

  //  vacuum_pump_pin.Write(true);
}

void GetBlock::Get_600Block()
{
  appstate = STATE_GET600BLOCK;
  rolldmmotor.SetPosVel(-2.30383492f, 2.0f);

  float target_lift = blockheight_2_liftmotortargetpos[2];
  // 参数顺序：左伸缩, 右伸缩, 左吸吮, 右吸吮, 左抬升, 右抬升
  SetTargetState(130000.0f, 130000.0f, 0.0f, 0.0f, target_lift, target_lift);

  //  vacuum_pump_pin.Write(true);
}

void GetBlock::ReleaseBlock()
{
  appstate = STATE_RELEASEBLOCK;
  rolldmmotor.SetPosVel(-2.30383492f, 2.0f);

  // 保持当前抬升和吸吮位置不变 (target_state_pos[1][2]为抬升，[5][6]为吸吮)
  // 仅将左右伸缩 (Stretch) 缩回至 -800000.0f
  SetTargetState(-800000.0f, -800000.0f,
                 target_state_pos[5], target_state_pos[6],
                 target_state_pos[1], target_state_pos[2]);

  //  vacuum_pump_pin.Write(false);
  // release_air_pin.Write(false); // 如果放块时需要操作泄气阀，可以在这里解开注释
}

void GetBlock::Action_LiftToHeight(float height)
{
  // TODO: 根据 height（mm）换算 liftmotor (注意：不是 stretchmotor) 的 total_angle 目标值并下发
  (void)height;
}

// ======================== GetTargetBlockInfo ========================

/**
 * @brief 从遥控器解析当前帧可取 KFS 的坐标和高度，存入 target_block_pos
 */
void GetBlock::GetTargetBlockInfo()
{
  target_count = 0; // 每帧重新统计，避免累计

  for (int i = 0; i < 12; i++)
  {
    if (farcon.KFS_values[i] != 1)
      continue;

    // if (System.GetCamp() == Systems::Camp_Red)
    // {
    //     if (target_count >= 3) break;
    //     target_block_pos[target_count] = RED_ALL_BLOCKS_DATA[i];
    //     target_count++;
    // }
    // else if (System.GetCamp() == Systems::Camp_Blue)
    // {
    //     // TODO: 蓝方块坐标数据
    // }
  }
}