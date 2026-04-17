#ifndef _CHASSIS_HPP_
#define _CHASSIS_HPP_

#include "std_math.hpp"
// #include "motor_dji.hpp"
#include "System.hpp"



#define ROTATE_RADIUS 0.446286f
#define WHEEL_DIAMETER 0.125133f

class MoveAct;
class ChassisType;

/**
 * @brief 路径类，包含了一系列路径点
 */
class Path
{
    Vec2 key_points[32];            // 路径点数组
};

// 不同于汽车的档位，这个档位是为了判别空挡并重启，以及设置底盘不同速度性能参数
typedef enum 
{
    NEUTRAL = 0,
    FIRST = 1,
    SECOND = 2,
    THIRD = 3
}Gear;

class ChasAPIHandle
{
    public:
    ChasAPIHandle(){};
    
    bool Reached = false;     // 是否到达目标位置
};

/**
 * @brief 底盘类
 * @note 底盘类中大部分是实现类的接口，更多复杂的算法都不在这里
 * 这里只提供一些简单的运动控制接口
 */
class ChassisType : public Application
{
    SINGLETON(ChassisType) : Application("chassis") {};
    APPLICATION_OVERRIDE
    private:
        /*<     底盘速度与加速度限制    >*/
        /// @param 最大加速度，单位m/s^2
        float _max_accel = 4.0f;       
        /// @param 最大线速度，单位m/s
        float _max_velo = 0.5f;
        /// @param 最大角速度，单位rad/s
        float _max_omega = 0.5f;
        /// @param 最大角加速度，单位rad/s^2
        float _max_beta = 2.0f;

        /*<     控制相关标志位    >*/
        bool _walking = false;              // 是否正MoveAt
        bool _rotating = false;             // 是否正RotateTo
        bool _is_pos_locked = false;        // 常锁位置环
        bool _is_yaw_locked = false;        // 常锁姿态环

        Gear _cur_gear = NEUTRAL;           // 当前档位
        
        /// @param 四个电机的目标速度，单位m/s
        float _motor_spd[4];
        
        /// @param 安全锁定计时器
        int _safe_lock_tick = 0;
        
        /// @brief 发送到电机
        inline void _SendSpdToMotor(); 

        /// @brief 底盘速度上传
        void _UploadSpeed();

        /// @brief 底盘自解算里程计更新函数
        void _UpdateChasOdom();

        /// @brief 走到某点具体实现
        bool _Walking();
        /// @brief 转到某角度具体实现
        bool _Rotating();


        Vec3 targ_ges = Vec3(0, 0, 0);      // 期望姿态，车体右手系，x向前，y向左，z从上向下看逆时针，单位m，rad
        Vec3 targ_speed;                    // 期望速度，车体右手系，x向前，y向左，z左转，单位m/s，rad/s

        typedef struct
        {
            Vec3 pos;
            Vec3 speed;         // 自解算的速度向量
            float velocity;     // 对应的线速度，单位m/s
        }_SelfResoOdom;

        /// @brief 底盘控制模式
        typedef enum
        {
            API,        // 由电控API控制
            HOST,       // 由工控机控制
            FARCON,     // 由遥控器控制
            OPEN,       // 开放控制（直接Move控制）
        }_ChasConMode;

    public:
        // 分别对应四个底盘电机
        MotorDJI motors[4];

        // 属性参数
        float velo;     // 对应的线速度，单位m/s
        bool enabled = false;    // 底盘使能标志
        float targ_velo = 0;

        /// @param 底盘里程计
        _SelfResoOdom chas_odom;

        /// @param 控制模式
        _ChasConMode control_mode = OPEN;

        float move_precision = 0.05f;       // 最小移动精度，单位m
        float rotate_precision = 0.02f;     // 最小旋转精度，单位rad    （0.017453 rad / 度）

        // 是否到达目标位置
        bool move_ok = false;
        // 是否旋转到目标角度
        bool rotate_ok = false;

        /**         直接接口    (Direct)        **/
        void Config();
        void Enable();
        void Disable(); 
        /// @brief 直接设置底盘速度（一个通用的开环行为）
        void Move(Vec3 Spd);
        void Move(Vec2 Spd);
        void Rotate(float omega);

        void MoveAt(Vec2 Pos);
        void RotateAt(float yaw);

        // 获取底盘当前档位
        Gear GetGear();
        void SetGear(Gear gear);
        void SetSpeedParams(float max_accel, float max_velo, float max_omega, float max_beta);

        static void ChassisSpeedRxCallback(uint8_t task_id, const uint8_t* payload, uint8_t payload_len, void* user_ctx);

};


#endif