#include "button_gd32.h"
#include "main.h"

bool Button::ReadButtonPinIO(uint8_t _id)
{
    switch (_id)
    {
        case 1:
            return gpio_input_bit_get(BUTTON1_GPIO_Port, BUTTON1_Pin) == SET;
#ifdef BUTTON2_GPIO_Port
        case 2:
            return gpio_input_bit_get(BUTTON2_GPIO_Port, BUTTON2_Pin) == SET;
#endif
        default:
            return false;
    }
}
bool Button::IsPressed()
{
    return !ReadButtonPinIO(id);
}
