#pragma once

#include <cstdint>
#include "std_math.hpp"

namespace BSP
{
    namespace SPI
    {
        // 不透明指针替代 SPI_HandleTypeDef*
        struct OpaqueSpi;
        using SpiID = OpaqueSpi*;

        /**
         * @brief SPI 总线设备从机侧类
         * @note 每个实例代表挂载在SPI总线上的单个从设备
         */
        class Device
        {
        public:
            Device() = default;
            
            // 构造函数：指定所挂载的SPI句柄及CS引脚
            Device(SpiID spi, Pin cs_pin = {'\0', 0});

            void Init(SpiID spi, Pin cs_pin = {'\0', 0});

            void Transmit(uint8_t* tx_data, uint16_t size);
            void Receive(uint8_t* rx_data, uint16_t size);
            void TransRecv(uint8_t* tx_data, uint8_t* rx_data, uint16_t size);
            
            SpiID spi_id;

        private:
            void* cs_port_;     // 内部使用void*保存GPIO_TypeDef*，对外隐藏
            uint16_t cs_pin_;   // 保存解码后的GPIO_PIN_X
            bool has_cs_;
        };
    }
}
