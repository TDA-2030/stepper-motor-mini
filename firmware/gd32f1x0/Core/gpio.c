#include "main.h"

void gpio_config()
{
    gpio_mode_set(LED1_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED1_Pin | LED2_Pin);
    gpio_output_options_set(LED1_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, LED1_Pin | LED2_Pin);
    gpio_bit_set(LED1_GPIO_Port, LED1_Pin | LED2_Pin);

    gpio_mode_set(RS486_RE_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RS486_RE_Pin);
    gpio_output_options_set(RS486_RE_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, RS486_RE_Pin);
    gpio_bit_reset(RS486_RE_GPIO_Port, RS486_RE_Pin);

    gpio_mode_set(BUTTON1_GPIO_Port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BUTTON1_Pin);
}


void gpio_rs485_enable_send(uint8_t enable)
{
    gpio_bit_write(RS486_RE_GPIO_Port, RS486_RE_Pin, enable?SET:RESET);

}