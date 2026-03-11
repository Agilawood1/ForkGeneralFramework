#include "bsp_spi.hpp"
#include "bsp_halport.hpp"

namespace BSP
{
    namespace SPI
    {
        Device::Device(SpiID spi, Pin cs_pin)
        {
            this->Init(spi, cs_pin);
        }

        void Device::Init(SpiID spi, Pin cs_pin)
        {
            this->spi_id = spi;

            if (cs_pin.port == '\0' || cs_pin.port == 0) {
                this->has_cs_ = false;
                this->cs_port_ = nullptr;
                this->cs_pin_ = 0;
            } else {
                this->has_cs_ = true;
                this->cs_pin_ = (1 << cs_pin.number);

                switch (cs_pin.port) {
#ifdef GPIOA
                    case 'A': this->cs_port_ = (void*)GPIOA; break;
#endif
#ifdef GPIOB
                    case 'B': this->cs_port_ = (void*)GPIOB; break;
#endif
#ifdef GPIOC
                    case 'C': this->cs_port_ = (void*)GPIOC; break;
#endif
#ifdef GPIOD
                    case 'D': this->cs_port_ = (void*)GPIOD; break;
#endif
#ifdef GPIOE
                    case 'E': this->cs_port_ = (void*)GPIOE; break;
#endif
#ifdef GPIOF
                    case 'F': this->cs_port_ = (void*)GPIOF; break;
#endif
#ifdef GPIOG
                    case 'G': this->cs_port_ = (void*)GPIOG; break;
#endif
#ifdef GPIOH
                    case 'H': this->cs_port_ = (void*)GPIOH; break;
#endif
#ifdef GPIOI
                    case 'I': this->cs_port_ = (void*)GPIOI; break;
#endif
                    default: this->cs_port_ = nullptr; this->has_cs_ = false; break;
                }
            }

            // 初始化时拉高片选，保证总线空闲
            if (this->has_cs_ && this->cs_port_ != nullptr) {
#ifdef USE_REAL_HAL
                HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
            }
        }

        void Device::Transmit(uint8_t* tx_data, uint16_t size)
        {
#ifdef USE_REAL_HAL
            if (this->has_cs_)
                HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);
                
            HAL_SPI_Transmit((SPI_HandleTypeDef*)this->spi_id, tx_data, size, 100);
            
            if (this->has_cs_)
                HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
        }

        void Device::Receive(uint8_t* rx_data, uint16_t size)
        {
#ifdef USE_REAL_HAL
            if (this->has_cs_)
                HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);
                
            HAL_SPI_Receive((SPI_HandleTypeDef*)this->spi_id, rx_data, size, 100);
            
            if (this->has_cs_)
                HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
        }

        void Device::TransRecv(uint8_t* tx_data, uint8_t* rx_data, uint16_t size)
        {
#ifdef USE_REAL_HAL
            if (this->has_cs_)
                HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_RESET);
                
            HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)this->spi_id, tx_data, rx_data, size, 100);
            
            if (this->has_cs_)
                HAL_GPIO_WritePin((GPIO_TypeDef*)this->cs_port_, this->cs_pin_, GPIO_PIN_SET);
#endif
        }
    }
}
