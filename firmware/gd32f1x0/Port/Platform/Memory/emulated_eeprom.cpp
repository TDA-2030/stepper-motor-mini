#ifndef __GD32_EEPROM_HPP
#define __GD32_EEPROM_HPP

#include <string.h>
#include "emulated_eeprom.h"
#include "stockpile.h"

#ifdef __cplusplus
extern "C" {
#endif


#if !defined(DATA_EEPROM_BASE)
static uint8_t eeprom_buffer[Stockpile_Page_Size] __attribute__((aligned(8))) = {0};
#endif

/**
  * @brief  Function reads a byte from emulated eeprom (flash)
  * @param  pos : address to read
  * @retval byte : data read from eeprom
  */
uint8_t eeprom_read_byte(const uint32_t pos)
{
#if defined(DATA_EEPROM_BASE)
    __IO uint8_t data = 0;
    if (pos <= (DATA_EEPROM_END - DATA_EEPROM_BASE)) {
      /* with actual EEPROM, pos is a relative address */
      data = *(__IO uint8_t *)(DATA_EEPROM_BASE + pos);
    }
    return (uint8_t)data;
#else
    eeprom_buffer_fill();
    return eeprom_buffered_read_byte(pos);
#endif /* _EEPROM_BASE */
}

/**
  * @brief  Function writes a byte to emulated eeprom (flash)
  * @param  pos : address to write
  * @param  value : value to write
  * @retval none
  */
void eeprom_write_byte(uint32_t pos, uint8_t value)
{
#if defined(DATA_EEPROM_BASE)
    /* with actual EEPROM, pos is a relative address */
    if (pos <= (DATA_EEPROM_END - DATA_EEPROM_BASE)) {
      if (HAL_FLASHEx_DATAEEPROM_Unlock() == HAL_OK) {
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (pos + DATA_EEPROM_BASE), (uint32_t)value);
        HAL_FLASHEx_DATAEEPROM_Lock();
      }
    }
#else
    eeprom_buffered_write_byte(pos, value);
    eeprom_buffer_flush();
#endif /* _EEPROM_BASE */
}

#if !defined(DATA_EEPROM_BASE)

/**
  * @brief  Function reads a byte from the eeprom buffer
  * @param  pos : address to read
  * @retval byte : data read from eeprom
  */
uint8_t eeprom_buffered_read_byte(const uint32_t pos)
{
    return eeprom_buffer[pos];
}

/**
  * @brief  Function writes a byte to the eeprom buffer
  * @param  pos : address to write
  * @param  value : value to write
  * @retval none
  */
void eeprom_buffered_write_byte(uint32_t pos, uint8_t value)
{
    eeprom_buffer[pos] = value;
}

/**
  * @brief  This function copies the data from flash into the buffer
  * @param  none
  * @retval none
  */
void eeprom_buffer_fill(void)
{
    memcpy(eeprom_buffer, (uint8_t*) (stockpile_data.begin_add), stockpile_data.area_size);
}

#if defined(EEPROM_RETRAM_MODE)

/**
  * @brief  This function writes the buffer content into the flash
  * @param  none
  * @retval none
  */
void eeprom_buffer_flush(void)
{
  memcpy((uint8_t *)(FLASH_BASE_ADDRESS), eeprom_buffer, E2END + 1);
}

#else /* defined(EEPROM_RETRAM_MODE) */

/**
  * @brief  This function writes the buffer content into the flash
  * @param  none
  * @retval none
  */
void eeprom_buffer_flush(void)
{
    Stockpile_Flash_Data_Empty(&stockpile_data);
    Stockpile_Flash_Data_Begin(&stockpile_data);
    
    Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t*)&eeprom_buffer, sizeof(eeprom_buffer)/4);
    
    Stockpile_Flash_Data_End(&stockpile_data);
}

#endif /* defined(EEPROM_RETRAM_MODE) */

#endif /* ! DATA_EEPROM_BASE */

#ifdef __cplusplus
}
#endif
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
