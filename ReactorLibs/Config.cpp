#include "bsp_halport.hpp"
#include "bsp_hardware.hpp"

/**
 * @brief 硬件与框架的 映射链接处 
 * @note 请在此处映射所有的 硬件 到框架内部
 */
void Hardware::Config_Hardwares()
{
    /**-----    配置CAN总线     -----**/
    Hardware::hcan_main = ToID(&hcan1);
    Hardware::hcan_sub = ToID(&hcan2);
 
    /**-----    配置串口    -----**/
    Hardware::huart_host = ToID(&huart1);
    Hardware::huart_farcon = ToID(&huart3);
    Hardware::huart_odom = ToID(&huart6);
    Hardware::huart_other = nullptr;
 
    /**-----    配置SPI总线    -----**/
    Hardware::spi_main_bus = nullptr;
    Hardware::spi_imu_bus = nullptr;
    Hardware::spi_ext_bus = nullptr;

    /**-----    配置定时器    -----**/
    Hardware::htim_led = nullptr;
    // Hardware::htim_liftservo = &htim5;
    /**-----    配置 GPIO   -----**/

    Hardware::Config_Parameters();
}

/**
 * @brief 框架参数配置函数
 * @note 请在此处配置 所用硬件基底的参数
 */
void Hardware::Config_Parameters()
{
    /// @brief 配置主频
    Hardware::MainFreq_MHz = 180;

    /// @brief 日志输出模式
    Hardware::RTTLogAtUart = true;
}
