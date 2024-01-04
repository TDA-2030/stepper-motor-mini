/**************************************************************************//**
  \file     spi.h
  \brief    this is the header file of spi.c.
  \author   Lao·Zhu
  \version  V1.0.1
  \date     10. October 2021
 ******************************************************************************/

#ifndef MINIFOC_HARDWARE_SPI_H_
#define MINIFOC_HARDWARE_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

unsigned short spi_readwrite_halfworld(unsigned short data);
void spi_config(void);
void spi_cs_enable();
void spi_cs_disable();


#ifdef __cplusplus
}
#endif

#endif //MINIFOC_HARDWARE_SPI_H_
