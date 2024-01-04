
#ifndef MINIFOC_HARDWARE_GPIO_H_
#define MINIFOC_HARDWARE_GPIO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


void gpio_config();

void gpio_rs485_enable_send(uint8_t enable);


#ifdef __cplusplus
}
#endif

#endif //MINIFOC_HARDWARE_GPIO_H_
