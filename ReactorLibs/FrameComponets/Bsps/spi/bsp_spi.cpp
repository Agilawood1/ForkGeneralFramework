#include "bsp_spi.hpp"
#include "bsp_halport.hpp"

namespace BSP
{
namespace SPI
{

static constexpr uint8_t MAX_SPI_BUS_NUM = 6;
struct SpiBusState {
    SpiID spi_id;
    Device* active_device;
    DmaState dma_state;
};
static SpiBusState spi_buses[MAX_SPI_BUS_NUM];
static uint8_t spi_buses_count = 0;

static void SetActiveDevice(SpiID id, Device* dev) {
    for (uint8_t i = 0; i < spi_buses_count; i++) {
        if (spi_buses[i].spi_id == id) {
            spi_buses[i].active_device = dev;
            return;
        }
    }
    if (spi_buses_count < MAX_SPI_BUS_NUM) {
        spi_buses[spi_buses_count].spi_id = id;
        spi_buses[spi_buses_count].active_device = dev;
        spi_buses[spi_buses_count].dma_state = DmaState::Idle;
        spi_buses_count++;
    }
}

static Device* GetActiveDevice(SpiID id) {
    for (uint8_t i = 0; i < spi_buses_count; i++) {
        if (spi_buses[i].spi_id == id) {
            return spi_buses[i].active_device;
        }
    }
    return nullptr;
}

static void SetDmaState(SpiID id, DmaState state) {
    for (uint8_t i = 0; i < spi_buses_count; i++) {
        if (spi_buses[i].spi_id == id) {
            spi_buses[i].dma_state = state;
            return;
        }
    }
    if (spi_buses_count < MAX_SPI_BUS_NUM) {
        spi_buses[spi_buses_count].spi_id = id;
        spi_buses[spi_buses_count].active_device = nullptr;
        spi_buses[spi_buses_count].dma_state = state;
        spi_buses_count++;
    }
}

static DmaState GetDmaState(SpiID id) {
    for (uint8_t i = 0; i < spi_buses_count; i++) {
        if (spi_buses[i].spi_id == id) {
            return spi_buses[i].dma_state;
        }
    }
    return DmaState::Idle;
}

Device::Device(SpiID spi, Pin cs_pin)
{
  this->Init(spi, cs_pin);
}

void Device::Init(SpiID spi, Pin cs_pin)
{
  this->spi_id = spi;

  if (cs_pin.port == '\0' || cs_pin.port == 0)
  {
    this->has_cs_ = false;
    this->cs_port_ = nullptr;
    this->cs_pin_ = 0;
  }
  else
  {
    this->has_cs_ = true;
    this->cs_pin_ = (1 << cs_pin.number);

    switch (cs_pin.port)
    {
#ifdef GPIOA
      case 'A':
        this->cs_port_ = (void *)GPIOA;
        break;
#endif
#ifdef GPIOB
      case 'B':
        this->cs_port_ = (void *)GPIOB;
        break;
#endif
#ifdef GPIOC
      case 'C':
        this->cs_port_ = (void *)GPIOC;
        break;
#endif
#ifdef GPIOD
      case 'D':
        this->cs_port_ = (void *)GPIOD;
        break;
#endif
#ifdef GPIOE
      case 'E':
        this->cs_port_ = (void *)GPIOE;
        break;
#endif
#ifdef GPIOF
      case 'F':
        this->cs_port_ = (void *)GPIOF;
        break;
#endif
#ifdef GPIOG
      case 'G':
        this->cs_port_ = (void *)GPIOG;
        break;
#endif
#ifdef GPIOH
      case 'H':
        this->cs_port_ = (void *)GPIOH;
        break;
#endif
#ifdef GPIOI
      case 'I':
        this->cs_port_ = (void *)GPIOI;
        break;
#endif
      default:
        this->cs_port_ = nullptr;
        this->has_cs_ = false;
        break;
    }
  }

  // 初始化时拉高片选，保证总线空闲
  if (this->has_cs_ && this->cs_port_ != nullptr)
  {
#ifdef USE_REAL_HAL
    HAL_GPIO_WritePin((GPIO_TypeDef *)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
  }
}

void Device::Transmit(uint8_t *tx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
  if (this->has_cs_)
    HAL_GPIO_WritePin((GPIO_TypeDef *)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);

  HAL_SPI_Transmit((SPI_HandleTypeDef *)this->spi_id, tx_data, size, 100);

  if (this->has_cs_)
    HAL_GPIO_WritePin((GPIO_TypeDef *)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
}

void Device::Receive(uint8_t *rx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
  if (this->has_cs_)
    HAL_GPIO_WritePin((GPIO_TypeDef *)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);

  HAL_SPI_Receive((SPI_HandleTypeDef *)this->spi_id, rx_data, size, 100);

  if (this->has_cs_)
    HAL_GPIO_WritePin((GPIO_TypeDef *)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
}

void Device::TransRecv(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
  if (this->has_cs_)
    HAL_GPIO_WritePin((GPIO_TypeDef *)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);

  HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)this->spi_id, tx_data, rx_data, size, 100);

  if (this->has_cs_)
    HAL_GPIO_WritePin((GPIO_TypeDef *)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
}

void Device::Select()
{
#ifdef USE_REAL_HAL
  if (this->has_cs_ && this->cs_port_ != nullptr)
  {
    HAL_GPIO_WritePin((GPIO_TypeDef *)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);
  }
#endif
}

void Device::Deselect()
{
#ifdef USE_REAL_HAL
  if (this->has_cs_ && this->cs_port_ != nullptr)
  {
    HAL_GPIO_WritePin((GPIO_TypeDef *)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
  }
#endif
}

bool Device::TransmitDMA(uint8_t *tx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
  if (GetDmaState(this->spi_id) == DmaState::Busy)
    return false;

  this->Select();
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit_DMA((SPI_HandleTypeDef *)this->spi_id, tx_data, size);
  if (status == HAL_OK)
  {
    SetActiveDevice(this->spi_id, this);
    SetDmaState(this->spi_id, DmaState::Busy);
    return true;
  }

  this->Deselect();
  SetActiveDevice(this->spi_id, nullptr);
  SetDmaState(this->spi_id, DmaState::Error);
  return false;
#endif
  return false;
}

bool Device::ReceiveDMA(uint8_t *rx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
  if (GetDmaState(this->spi_id) == DmaState::Busy)
    return false;

  this->Select();
  HAL_StatusTypeDef status =
      HAL_SPI_Receive_DMA((SPI_HandleTypeDef *)this->spi_id, rx_data, size);
  if (status == HAL_OK)
  {
    SetActiveDevice(this->spi_id, this);
    SetDmaState(this->spi_id, DmaState::Busy);
    return true;
  }

  this->Deselect();
  SetActiveDevice(this->spi_id, nullptr);
  SetDmaState(this->spi_id, DmaState::Error);
  return false;
#endif
  return false;
}

bool Device::TransRecvDMA(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
#ifdef USE_REAL_HAL
  if (GetDmaState(this->spi_id) == DmaState::Busy)
    return false;

  this->Select();
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
      (SPI_HandleTypeDef *)this->spi_id, tx_data, rx_data, size);
  if (status == HAL_OK)
  {
    SetActiveDevice(this->spi_id, this);
    SetDmaState(this->spi_id, DmaState::Busy);
    return true;
  }

  this->Deselect();
  SetActiveDevice(this->spi_id, nullptr);
  SetDmaState(this->spi_id, DmaState::Error);
  return false;
#endif
  return false;
}

DmaState Device::ConsumeDmaState()
{
  DmaState state = GetDmaState(this->spi_id);
  if (state == DmaState::Done || state == DmaState::Error)
  {
    SetDmaState(this->spi_id, DmaState::Idle);
  }
  return state;
}

} // namespace SPI
} // namespace BSP

#ifdef USE_REAL_HAL
// 通用回调处理函数，用于自动拉高失活设备的 CS
static void SPI_DeselectActiveDevice(SPI_HandleTypeDef *hspi)
{
  BSP::SPI::Device* dev = BSP::SPI::GetActiveDevice((BSP::SPI::SpiID)hspi);
  if (dev != nullptr)
  {
    dev->Deselect();
  }
  BSP::SPI::SetActiveDevice((BSP::SPI::SpiID)hspi, nullptr); // 清除活跃设备绑定
}

// === HAL 层中断回调重载 ===

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  SPI_DeselectActiveDevice(hspi);
  BSP::SPI::SetDmaState((BSP::SPI::SpiID)hspi, BSP::SPI::DmaState::Done);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  SPI_DeselectActiveDevice(hspi);
  BSP::SPI::SetDmaState((BSP::SPI::SpiID)hspi, BSP::SPI::DmaState::Done);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  SPI_DeselectActiveDevice(hspi);
  BSP::SPI::SetDmaState((BSP::SPI::SpiID)hspi, BSP::SPI::DmaState::Done);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  SPI_DeselectActiveDevice(hspi);
  BSP::SPI::SetDmaState((BSP::SPI::SpiID)hspi, BSP::SPI::DmaState::Error);
}
#endif
