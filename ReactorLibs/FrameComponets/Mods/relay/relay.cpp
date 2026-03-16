/**
 * @author: Agilawood
 * @name: 继电器库 
 * @last_edit_time: 2026-1-25
 * @brief: 该库用于驱动继电器，把每个继电器通道看作一个对象
 */

#include "relay.hpp"

void Relay::Init(GPIO_TypeDef *GPIOx, uint32_t Pin, TriggerType trig_type)
{
    BspGpio_InstRegist(&_gpio_inst, GPIOx, Pin);
    _trig_type = trig_type;
    initialized = true;
    _Set(OFF);
}

void Relay::_Set(State state)
{
    if (!initialized) return;

    uint32_t pin_state;
    if (_trig_type == HIGH_ON)
    {
        pin_state = (state == ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    else 
    {
        pin_state = (state == ON) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    BspGpio_SetState(&_gpio_inst, pin_state);
    _state = state;
}

void Relay::On()
{
    _Set(ON);
}

void Relay::Off()
{
    _Set(OFF);
}


