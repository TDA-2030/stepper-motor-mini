/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32_EEPROM_H
#define __GD32_EEPROM_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint8_t eeprom_read_byte(uint32_t pos);
void eeprom_write_byte(uint32_t pos, uint8_t value);

#if !defined(DATA_EEPROM_BASE)
void eeprom_buffer_fill();
void eeprom_buffer_flush();
uint8_t eeprom_buffered_read_byte(uint32_t pos);
void eeprom_buffered_write_byte(uint32_t pos, uint8_t value);
#endif /* ! DATA_EEPROM_BASE */

#ifdef __cplusplus
}
#endif

#endif /* __GD32_EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
