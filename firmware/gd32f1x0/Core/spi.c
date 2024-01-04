/**************************************************************************//**
  \file     spi.c
  \brief    this file contains the initialization of SPI peripherals and
            the code implementation of reading and writing half words.
  \author   Lao·Zhu
  \version  V1.0.2
  \date     29. October 2021
 ******************************************************************************/

#include "spi.h"
#include "gd32f1x0.h"

/*!
    \brief      spi0 transmit data for sc60228
    \param[in]  data: data to transmit
    \retval     data received from slave
*/
unsigned short spi_readwrite_halfworld(unsigned short data) {
    unsigned short buffer;
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));

    /* send this data through spi0 */
    spi_i2s_data_transmit(SPI0, data);
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));

    /* get data received through spi0 */
    buffer = spi_i2s_data_receive(SPI0);
    return buffer;
}

void spi_cs_enable()
{
    gpio_bit_reset(GPIOA, GPIO_PIN_15);
}

void spi_cs_disable()
{
    gpio_bit_set(GPIOA, GPIO_PIN_15);
}

/*!
    \brief configure spi0 periph and its gpios
*/
void spi_config(void) {
    spi_parameter_struct spi_init_struct;

    /* enable GPIO clock and SPI0 clock*/
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI0);

    /* SPI0 GPIO config: SCK/PB3, MISO/PB4, MOSI/PB5 */
    gpio_af_set(GPIOB, GPIO_AF_0, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    /* SPI0 GPIO config: CS/PA15 */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);

    /* pull up CS pin to end sending data */
    gpio_bit_set(GPIOA, GPIO_PIN_15);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.prescale = SPI_PSC_8;
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_crc_off(SPI0);
    spi_init(SPI0, &spi_init_struct);

    /* SPI0 enable */
    spi_enable(SPI0);
}
