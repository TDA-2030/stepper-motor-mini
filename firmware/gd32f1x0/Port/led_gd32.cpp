#include "led_gd32.h"
#include "main.h"

void Led::SetLedState(uint8_t _id, bool _state)
{
    switch (_id)
    {
        case 0:
            gpio_bit_write(LED1_GPIO_Port, LED1_Pin, _state ? SET : RESET);
            break;
        case 1:
            gpio_bit_write(LED2_GPIO_Port, LED2_Pin, _state ? SET : RESET);
            break;

        default:
            break;
    }
}
