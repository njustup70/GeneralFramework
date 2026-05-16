#ifndef STD_MSG_UP70
#define STD_MSG_UP70

#include <cstdint>

#define BSP_SQRT2 1.41421356237f
#define BSP_SQRT3 1.73205080757f

typedef uint8_t byte;

/// @brief GPIO 引脚定义结构体
struct Pin
{
  char port;      // 'A', 'B', 'C', ..., 留空或 '\0' 表示无 CS 引脚
  uint8_t number; // 0 - 15
};

class Vec2;
class Vec3;

/**
 * @name 二维向量
 */
class Vec2
{
private:
  /* data */
public:
  float x, y;
  // 构造函数
  Vec2(float x = 0, float y = 0) : x(x), y(y)
  {
  }

  // 基本运算
  Vec2 Norm() const;    // 获取归一化向量
  float Length() const; // 获取向量长度
  float Angle() const;
  Vec2 Rotate(float angRad) const;
  Vec3 ToVec3() const;

  // 类型转换
  void ToBytes(uint8_t *buffer) const;         // 将Vec2转换入buffer
  void FromBytes(const uint8_t *buffer); // 从buffer恢复Vec2

  // 友元函数重载运算符
  friend Vec2 operator+(const Vec2 &lhs, const Vec2 &rhs);
  friend Vec2 operator-(const Vec2 &lhs, const Vec2 &rhs);
  friend float operator*(const Vec2 &lhs, const Vec2 &rhs);
  friend Vec2 operator*(const Vec2 &vec, float scalar);
  friend Vec2 operator*(float scalar, const Vec2 &vec);
  friend Vec2 operator/(const Vec2 &vec, float scalar);
  friend bool operator==(const Vec2 &lhs, const Vec2 &rhs);
  friend bool operator!=(const Vec2 &lhs, const Vec2 &rhs);
};
/*********      运算符重载      **********/
inline Vec2 operator+(const Vec2 &lhs, const Vec2 &rhs)
{ // 向量加法
  return Vec2(lhs.x + rhs.x, lhs.y + rhs.y);
}
inline Vec2 operator-(const Vec2 &lhs, const Vec2 &rhs)
{ // 向量减法
  return Vec2(lhs.x - rhs.x, lhs.y - rhs.y);
}
inline float operator*(const Vec2 &lhs, const Vec2 &rhs)
{ // 向量点乘
  return lhs.x * rhs.x + lhs.y * rhs.y;
}
inline Vec2 operator*(const Vec2 &vec, float scalar)
{ // 向量数乘（向量在前）
  return Vec2(vec.x * scalar, vec.y * scalar);
}
inline Vec2 operator*(float scalar, const Vec2 &vec)
{ // 向量数乘（标量在前）
  return Vec2(vec.x * scalar, vec.y * scalar);
}
inline Vec2 operator/(const Vec2 &vec, float scalar)
{ // 向量数除
  if (scalar == 0)
    return Vec2(114514, 114514);
  else
    return Vec2(vec.x / scalar, vec.y / scalar);
}
inline bool operator==(const Vec2 &lhs, const Vec2 &rhs)
{ // 向量相等比较
  return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}
inline bool operator!=(const Vec2 &lhs, const Vec2 &rhs)
{ // 向量不等比较
  return !(lhs == rhs);
}

/**
 * @name 三维向量
 */
class Vec3
{
private:
  /* data */
public:
  float x, y, z;
  // 构造函数的实现直接放在类定义中
  Vec3(float x, float y, float z) : x(x), y(y), z(z)
  {
  }
  Vec3() : x(0), y(0), z(0)
  {
  }
  Vec3 Norm() const;                   // 获取归一化向量
  float Length() const;                // 获取向量长度
  void ToBytes(uint8_t *buffer) const; // 将Vec3转换入buffer
  void FromBytes(const uint8_t *buffer);
  Vec2 ToVec2() const;

  // 友元函数重载运算符
  friend Vec3 operator+(const Vec3 &lhs, const Vec3 &rhs);
  friend Vec3 operator-(const Vec3 &lhs, const Vec3 &rhs);
  friend float operator*(const Vec3 &lhs, const Vec3 &rhs);
  friend Vec3 operator^(const Vec3 &lhs, const Vec3 &rhs);
  friend Vec3 operator*(const Vec3 &vec, float scalar);
  friend Vec3 operator*(float scalar, const Vec3 &vec);
  friend Vec3 operator/(const Vec3 &vec, float scalar);
};
/*********      运算符重载      **********/
inline Vec3 operator+(const Vec3 &lhs, const Vec3 &rhs)
{
  return Vec3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}
inline Vec3 operator-(const Vec3 &lhs, const Vec3 &rhs)
{
  return Vec3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}
inline float operator*(const Vec3 &lhs, const Vec3 &rhs)
{ // 向量点乘
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}
inline Vec3 operator^(const Vec3 &lhs, const Vec3 &rhs)
{ // 向量叉乘
  return Vec3(lhs.y * rhs.z - lhs.z * rhs.y,
              lhs.z * rhs.x - lhs.x * rhs.z,
              lhs.x * rhs.y - lhs.y * rhs.x);
}
inline Vec3 operator*(const Vec3 &vec, float scalar)
{ // 向量数乘（向量在前）
  return Vec3(vec.x * scalar, vec.y * scalar, vec.z * scalar);
}
inline Vec3 operator*(float scalar, const Vec3 &vec)
{ // 向量数乘（标量在前）
  return Vec3(vec.x * scalar, vec.y * scalar, vec.z * scalar);
}
inline Vec3 operator/(const Vec3 &vec, float scalar)
{ // 向量数除
  if (scalar == 0)
    return Vec3(114514, 114514, 114514); // 避免除以零
  else
    return Vec3(vec.x / scalar, vec.y / scalar, vec.z / scalar);
}

/**
 * @name 三维向量
 * @warning 颜色分量虽然是float类型，但其适配RGB24方案，取值范围应为0.0f~255.0f
 */
class Color
{
private:
  /* data */
public:
  float r, g, b;
  // 构造函数的实现直接放在类定义中
  Color(float r, float g, float b) : r(r), g(g), b(b)
  {
  }
  Color() : r(0), g(0), b(0)
  {
  }

  // 预定义颜色
  static Color Red;
  static Color Green;
  static Color Blue;
  static Color White;

  // 友元函数重载运算符
  friend Color operator+(const Color &lhs, const Color &rhs);
  friend Color operator-(const Color &lhs, const Color &rhs);
  friend Color operator*(const Color &vec, float scalar);
  friend Color operator*(const Color &vec, Vec3 vecscalar);
  friend Color operator*(float scalar, const Color &vec);
  friend Color operator/(const Color &vec, float scalar);
};
/*********      运算符重载      **********/
inline Color operator+(const Color &lhs, const Color &rhs)
{
  return Color(lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b);
}
inline Color operator-(const Color &lhs, const Color &rhs)
{
  return Color(lhs.r - rhs.r, lhs.g - rhs.g, lhs.b - rhs.b);
}
inline Color operator*(const Color &vec, float scalar)
{ // 向量数乘（向量在前）
  return Color(vec.r * scalar, vec.g * scalar, vec.b * scalar);
}
inline Color operator*(const Color &vec, Vec3 vecscalar)
{ // 颜色哈达玛积
  return Color(vec.r * vecscalar.x, vec.g * vecscalar.y, vec.b * vecscalar.z);
}
inline Color operator*(float scalar, const Color &vec)
{ // 向量数乘（标量在前）
  return Color(vec.r * scalar, vec.g * scalar, vec.b * scalar);
}
inline Color operator/(const Color &vec, float scalar)
{ // 向量数除
  if (scalar == 0)
    return Color(114514, 114514, 114514); // 避免除以零
  return Color(vec.r / scalar, vec.g / scalar, vec.b / scalar);
}

/**
 * @name 四元数
 */
class Quat
{
public:
  float w, x, y, z;
  Quat(float w = 1.0f, float x = 0.0f, float y = 0.0f, float z = 0.0f) : w(w), x(x), y(y), z(z)
  {
  }

  void Normalize();
  Vec3 ToEuler() const;

  friend Quat operator*(const Quat &lhs, const Quat &rhs);
};

/*********      运算符重载      **********/
inline Quat operator*(const Quat &lhs, const Quat &rhs)
{
  return Quat(
      lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
      lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
      lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w);
}

typedef struct
{
  int id;
  float error;
  float distan;
  float theta;
} SickData;

typedef struct
{
  float x;
  float y;
  float fai;
} Vec3_Odom;

/**
 * @name 发射信息
 * @brief
 * @param Rpm: 发射时设定的Rpm
 * @param Delta_Up: 上轮的减速情况
 * @param Delta_Down: 下轮的减速情况
 * @param Velo：本次的出膛速度
 * @param Result：本次是否成功 0 成功，1 过大，-1 过小
 */
typedef struct ShootInfo
{
  float Rpm;
  float Delta_Up;
  float Delta_Down;
  float Velo;
  int8_t Result;
} ShootInfo;

/*********      限幅     **********/
/**
 * @name lim_abs
 * @brief 限幅函数
 * @param targ_num: 待限幅的数值
 * @param limit_mx: 限幅的绝对值上限
 */
float Limit_ABS(float targ_num, float limit_mx);

namespace StdMath
{
/// @brief 转速转弧度速度
/// @param rpm 转速 (RPM)
/// @return 弧度速度
float RpmToRadS(float rpm);

/// @brief 弧度速度转转速
/// @param rad_s
/// @return
float RadSToRpm(float rad_s);

/// @brief 限幅函数
/// @param val 目标值
/// @param limit 限幅值
float fclamp(float val, float limit);

/// @brief 通用限幅函数
/// @param val 目标值
/// @param min_val 下限
/// @param max_val 上限
float fclamp(float val, float min_val, float max_val);

/// @brief 三维向量叉乘
/// @param a 向量a
/// @param b 向量b
/// @return a x b
Vec3 Cross3(const Vec3 &a, const Vec3 &b);

/// @brief 符号函数
/// @param val 目标值
/// @return 目标值的符号 + / -
int signf(float val);

// float 4字节->2字节压缩函数
void PackFloatToInt16(uint8_t* ptr, float val, float scale);
float UnpackInt16ToFloat(const uint8_t* ptr, float scale);

} // namespace StdMath



#endif