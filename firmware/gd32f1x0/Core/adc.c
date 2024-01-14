
/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "math.h"
#include "systick.h"

static uint16_t whole_adc_data[2];

/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void dma_config(void)
{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    /* ADC DMA_channel configuration */
    dma_deinit(DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA);
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&whole_adc_data);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = 2;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA_CH0);
    /* enable DMA channel */
    dma_channel_enable(DMA_CH0);
}

/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{
    rcu_periph_clock_enable(RCU_DMA);
    rcu_periph_clock_enable(RCU_ADC);
    rcu_periph_clock_enable(RCU_GPIOA);

    dma_config();

    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);

    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1);

    /* ADC continuous function enable */
    adc_special_function_config(ADC_CONTINUOUS_MODE, ENABLE);
    /* ADC scan function enable */
    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 2);

    /* ADC regular channel config */
    adc_regular_channel_config(0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(1, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);

    /* ADC DMA function enable */
    adc_dma_mode_enable();
    /* enable ADC interface */
    adc_enable();
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable();

    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
}


#define TEMP_GROUP 1
const float BALANCE_RESISTOR   = 3300.0;
// This helps calculate the thermistor's resistance (check article for details).
const float MAX_ADC            = 4095.0;
/* This is thermistor dependent and it should be in the datasheet, or refer to the
   article for how to calculate it using the Beta equation.
   I had to do this, but I would try to get a thermistor with a known
   beta if you want to avoid empirical calculations. */
const float BETA               = 3455.0;
/* This is also needed for the conversion equation as "typical" room temperature
   is needed as an input. */
const float ROOM_TEMP          = 298.15;   // room temperature in Kelvin
/* Thermistors will have a typical resistance at room temperature so write this
   down here. Again, needed for conversion equations. */
const float RESISTOR_ROOM_TEMP = 10000.0;
/* USER CODE BEGIN 1 */

float AdcGetChipTemperature()
{
    float rThermistor = 0;            // Holds thermistor resistance value
    float tKelvin     = 0;            // Holds calculated temperature
    float tempVal = 0;
    float adcVal = (float) (whole_adc_data[TEMP_GROUP] + 1);

    rThermistor = BALANCE_RESISTOR * ( (MAX_ADC / adcVal) - 1);
    tKelvin = (BETA * ROOM_TEMP) /
              (BETA + (ROOM_TEMP * log(rThermistor / RESISTOR_ROOM_TEMP)));

    tempVal = tKelvin - 273.15;  // convert kelvin to celsius

    return tempVal;
}

float AdcGetVoltage()
{
    float tempVal = 0;
    float adcVal = (float) whole_adc_data[0];
    // 10k 1k
    tempVal = adcVal * 3.3 * 11 / 4095.0; 

    return tempVal;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
