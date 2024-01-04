
//Oneself
#include "stockpile.h"
#include "gd32f1x0.h"
////LL_Driver
//#include "GD32f0xx_ll_flash_ex.h"

////LL库FLASH清除函数
//extern void FLASH_PageErase(uint32_t PageAddress);

/*************************************************************** Flash_Start ***************************************************************/
/*************************************************************** Flash_Start ***************************************************************/
/*************************************************************** Flash_Start ***************************************************************/
//Flash分区表实例                                    起始地址                    ， 大小                     ， 页数                                                ， 写地址（中间变量）
Stockpile_FLASH_Typedef	stockpile_app_firmware	= {STOCKPILE_APP_FIRMWARE_ADDR, STOCKPILE_APP_FIRMWARE_SIZE,	(STOCKPILE_APP_FIRMWARE_SIZE / Stockpile_Page_Size),	0};
Stockpile_FLASH_Typedef	stockpile_quick_cali		= {STOCKPILE_APP_CALI_ADDR, 		STOCKPILE_APP_CALI_SIZE,			(STOCKPILE_APP_CALI_SIZE / Stockpile_Page_Size),			0};
Stockpile_FLASH_Typedef stockpile_data					= {STOCKPILE_APP_DATA_ADDR,			STOCKPILE_APP_DATA_SIZE, 			(STOCKPILE_APP_DATA_SIZE / Stockpile_Page_Size),			0};

/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fmc_erase_pages(uint32_t FMC_WRITE_START_ADDR, uint16_t PageNum)
{
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    
    /* erase the flash pages */
    for(int EraseCounter = 0U; EraseCounter < PageNum; EraseCounter++){
        fmc_page_erase(FMC_WRITE_START_ADDR + (Stockpile_Page_Size * EraseCounter));
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}


/**
  * @brief  Flash数据清空
  * @param  stockpile	Flash分区表实例
  * @retval NULL
**/
void Stockpile_Flash_Data_Empty(Stockpile_FLASH_Typedef *stockpile)
{
    fmc_erase_pages(stockpile->begin_add, stockpile->page_num);
}


/**
  * @brief  Flash数据开始写入
  * @param  stockpile	Flash分区表实例
  * @retval NULL
**/
void Stockpile_Flash_Data_Begin(Stockpile_FLASH_Typedef *stockpile)
{
	  fmc_unlock();
	  stockpile->asce_write_add = stockpile->begin_add;
}

/**
  * @brief  Flash数据结束写入
  * @param  stockpile	Flash分区表实例
  * @retval NULL
**/
void Stockpile_Flash_Data_End(Stockpile_FLASH_Typedef *stockpile)
{
    fmc_lock();
}

/**
  * @brief  Flash设置写地址
  * @param  stockpile	Flash分区表实例
  * @param  write_add	写地址
  * @retval NULL
**/
void Stockpile_Flash_Data_Set_Write_Add(Stockpile_FLASH_Typedef *stockpile, uint32_t write_add)
{
	if(write_add < stockpile->begin_add)						return;
	if(write_add > stockpile->begin_add + stockpile->area_size)	return;
	stockpile->asce_write_add = write_add;
}

/**
  * @brief  Flash_16位数据写入
  * @param  stockpile	Flash分区表实例
  * @param  data		半字数据缓冲区
  * @param  num			半字数量
  * @retval NULL
**/
void Stockpile_Flash_Data_Write_Data16(Stockpile_FLASH_Typedef *stockpile, uint16_t *data, uint32_t num)
{
	if(stockpile->asce_write_add < stockpile->begin_add)									return;
	if((stockpile->asce_write_add + num * 2) > stockpile->begin_add + stockpile->area_size)	return;
	
	for(uint32_t i=0; i<num; i++)
	{
		if(fmc_halfword_program(stockpile->asce_write_add, (uint16_t)data[i]) == FMC_READY)
			stockpile->asce_write_add += 2;
	}
}

/**
  * @brief  Flash_32位数据写入
  * @param  stockpile	Flash分区表实例
  * @param  data		字数据缓冲区
  * @param  num			字数量
  * @retval NULL
**/
void Stockpile_Flash_Data_Write_Data32(Stockpile_FLASH_Typedef *stockpile, uint32_t *data, uint32_t num)
{
	if(stockpile->asce_write_add < stockpile->begin_add)									return;
	if((stockpile->asce_write_add + num * 4) > stockpile->begin_add + stockpile->area_size)	return;
	
	for(uint32_t i=0; i<num; i++)
	{
		if(fmc_word_program(stockpile->asce_write_add, (uint32_t)data[i]) == FMC_READY)
			stockpile->asce_write_add += 4;
	}
}


/*************************************************************** Flash_End ***************************************************************/
/*************************************************************** Flash_End ***************************************************************/
/*************************************************************** Flash_End ***************************************************************/

