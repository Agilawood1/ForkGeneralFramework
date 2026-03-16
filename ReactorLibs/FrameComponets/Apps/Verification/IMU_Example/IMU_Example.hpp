#pragma once
#include <cstdint>
#include "System.hpp"
#include "bmi088.hpp"

/**
 * @brief BMI088 陀螺仪测试应用
 * @note 用于测试陀螺仪数据读取并打印到日志，封装自原 MainFrame.cpp 的测试代码。
 */
class IMU_Example : public Application
{
  SINGLETON(IMU_Example) : Application("IMU_Example") {};

  APPLICATION_OVERRIDE

private:
  enum class DmaStep : uint8_t
  {
    AccTrigger = 0,
    AccWait,
    GyroTrigger,
    GyroWait
  };

  BSP::SPI::Device spi_acc_inst_;
  BSP::SPI::Device spi_gyro_inst_;
  BMI088 imu_;
  DmaStep dma_step_ = DmaStep::AccTrigger;
  uint32_t tick_cnt_ = 0; // 更新计数器，用于降频打印
  uint32_t dma_fail_cnt_ = 0;
  uint32_t dma_pending_cnt_ = 0;
  uint32_t log_header_cnt_ = 0; // 日志表头计数器
};
