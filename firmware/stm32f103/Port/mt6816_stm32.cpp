#include "mt6816_stm32.h"
#include "spi.h"
#include "stm32f1xx_ll_spi.h"

void MT6816::SpiInit()
{
    MX_SPI1_Init();
    LL_SPI_Enable(SPI1);

}

static uint16_t SPI1_ReadWriteByte(uint16_t TxData)
{		
	uint32_t retry = 0;				 
	/* Check if Tx buffer is empty */
	// while (!LL_SPI_IsActiveFlag_TXE(SPI1)) 
	// {
	// 	retry++;
	// 	if(retry > 100) return 0;
	// }			  
 
	/* Write character in Data register.
	TXE flag is cleared by reading data in DR register */
	LL_SPI_TransmitData16(SPI1, TxData);
	retry = 0;
 
	/* Check if Rx buffer is not empty */
	while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) 
	{
		retry++;
		if(retry > 200) return 0;
	}	  						    
 
	/* received byte from SPI lines. */
	return LL_SPI_ReceiveData16(SPI1); 				    
}

uint16_t MT6816::SpiTransmitAndRead16Bits(uint16_t _dataTx)
{
    uint16_t dataRx;

    GPIOA->BRR = GPIO_PIN_15; // Chip select
    dataRx = SPI1_ReadWriteByte(_dataTx);
    // HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &_dataTx, (uint8_t*) &dataRx, 1, HAL_MAX_DELAY);
    GPIOA->BSRR = GPIO_PIN_15;

    return dataRx;
}
