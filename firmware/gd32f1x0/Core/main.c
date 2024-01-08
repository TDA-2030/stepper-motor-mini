
#include "main.h"
#include "systick.h"
#include "common_inc.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"

/*!
    \brief main function
*/
int main(void) {
    
    /* configure systick timer for delay_ms() function */
    systick_config();
    /* 4 bits for preemption priority 0 bits for subpriority */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /* configure peripherals */
    gpio_config();
    adc_config();
    uart_config();
    timer1_config();
    timer13_config();
    pwm_timer2_config();
    spi_config();

    Main();

    while (1) {
        
    }
}
