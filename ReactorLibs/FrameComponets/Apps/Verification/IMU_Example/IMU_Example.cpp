#include "IMU_Example.hpp"
#include "Monitor.hpp"
#include "spi.h"
#include "bsp_log.hpp"

// 0: 使用阻塞 SPI 测试；1: 使用 DMA SPI 测试
#ifndef IMU_EXAMPLE_USE_DMA
#define IMU_EXAMPLE_USE_DMA 0
#endif

void IMU_Example::Start()
{
  dma_step_ = DmaStep::AccTrigger;
  dma_fail_cnt_ = 0;
  dma_pending_cnt_ = 0;
  log_header_cnt_ = 20; // 首次运行时确保打印表头

  // 配置 SPI 实例: CS1_Accel -> PA4, CS1_Gyro -> PB0
  spi_acc_inst_.Init((BSP::SPI::SpiID)&hspi1, {'A', 4});
  spi_gyro_inst_.Init((BSP::SPI::SpiID)&hspi1, {'B', 0}); // 假设加速度计和陀螺仪共用SPI1，仅CS不同

  // 初始化 BMI088 (包含自动标定)
  imu_.Init(&spi_acc_inst_, &spi_gyro_inst_);

  Monitor &monit = Monitor::GetInstance();
  monit.Track(imu_.gyro.x);
  monit.Track(imu_.gyro.y);
  monit.Track(imu_.gyro.z);

#if IMU_EXAMPLE_USE_DMA
  BspLog_LogInfo("IMU_Example start in DMA SPI mode.");
#else
  BspLog_LogInfo("IMU_Example start in normal SPI mode.");
#endif
}

void IMU_Example::Update()
{
  Monitor &monit = Monitor::GetInstance();

  // 打印初始化状态
  if (!imu_.IsOnline())
  {
    monit.LogWarning(
        "IMU is offline! Init Failed. Acc_Err:0x%02X, Gyro_Err:0x%02X",
        imu_.GetInitError() >> 8, imu_.GetInitError() & 0xFF);
    return;
  }

  // 周期性更新数据
#if IMU_EXAMPLE_USE_DMA
  // 非阻塞 DMA 状态机：每次 Update 仅推进一步
  switch (dma_step_)
  {
  case DmaStep::AccTrigger:
    if (imu_.TriggerAccelDMA())
    {
      dma_step_ = DmaStep::AccWait;
    }
    else
    {
      dma_fail_cnt_++;
    }
    break;

  case DmaStep::AccWait:
  {
    BSP::SPI::DmaState state = imu_.ParseAccelDMA();
    if (state == BSP::SPI::DmaState::Done)
    {
      dma_step_ = DmaStep::GyroTrigger;
    }
    else if (state == BSP::SPI::DmaState::Error)
    {
      dma_fail_cnt_++;
      dma_step_ = DmaStep::AccTrigger;
    }
    else
    {
      dma_pending_cnt_++;
    }
    break;
  }

  case DmaStep::GyroTrigger:
    if (imu_.TriggerGyroDMA())
    {
      dma_step_ = DmaStep::GyroWait;
    }
    else
    {
      dma_fail_cnt_++;
      dma_step_ = DmaStep::AccTrigger;
    }
    break;

  case DmaStep::GyroWait:
  {
    BSP::SPI::DmaState state = imu_.ParseGyroDMA();
    if (state == BSP::SPI::DmaState::Done)
    {
      dma_step_ = DmaStep::AccTrigger;
    }
    else if (state == BSP::SPI::DmaState::Error)
    {
      dma_fail_cnt_++;
      dma_step_ = DmaStep::AccTrigger;
    }
    else
    {
      dma_pending_cnt_++;
    }
    break;
  }
  }
#else
  imu_.Update();
#endif

  tick_cnt_++;
  if (tick_cnt_ > 19)
  {
    tick_cnt_ = 0;

    // 定期打印表头以增强可读性
    if (log_header_cnt_ >= 20)
    {
      log_header_cnt_ = 0;
      monit.LogInfo("--- IMU DATA LOG (Unit: rad/s, m, rad) ---");
      monit.LogInfo("%-10s | %-7s %-7s %-7s | %-7s %-7s %-7s",
                    "Tag_G&P", "GyroX", "GyroY", "GyroZ", "PosX", "PosY", "PosZ");
      monit.LogInfo("%-10s | %-7s %-7s %-7s | %-7s %-7s",
                    "Tag_A&D", "AngX", "AngY", "AngZ", "DMAF", "DMAP");
      monit.LogInfo("---------------------------------------------------------");
    }

    log_header_cnt_++;

    // 获取数据
    const Vec3 &gyro = imu_.GetGyro();
    const Vec3 &odom_ang = imu_.GetOdomAngleRad();
    const Vec3 &odom_pos = imu_.GetOdomPositionM();

    // 拆分为两行打印，防止单行过长超出 Log 缓冲区。
    monit.LogInfo("%-10s | %7.3f %7.3f %7.3f | %7.3f %7.3f %7.3f",
                  "IMU_GP",
                  gyro.x, gyro.y, gyro.z,
                  odom_pos.x, odom_pos.y, odom_pos.z);
    monit.LogInfo("%-10s | %7.3f %7.3f %7.3f | %7lu %7lu",
                  "IMU_AD",
                  odom_ang.x, odom_ang.y, odom_ang.z,
                  dma_fail_cnt_, dma_pending_cnt_);
  }
}
