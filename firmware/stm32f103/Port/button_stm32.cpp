#include "button_stm32.h"
#include <gpio.h>

bool Button::ReadButtonPinIO(uint8_t _id)
{
    switch (_id)
    {
        case 1:
            return HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin) == GPIO_PIN_RESET;
        case 2:
            return HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin) == GPIO_PIN_RESET;
        default:
            return false;
    }
}
bool Button::IsPressed()
{
    return ReadButtonPinIO(id);
}
