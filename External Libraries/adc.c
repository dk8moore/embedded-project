/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "hw_msp.h"
#include "hw_gpio.h"
#include "stm32l0xx_hal_gpio.h"
#include "vcom.h"
#include <stdbool.h>

// Global variables -----------------------------------


/* ADC init function */
// BASIC INITIALIZATION OF THE ADC IS PERFORMED ON THE HW_INIT

// Methods --------------------------------------------

void ADC_PinInit(uint16_t pin)
{
  if(hadc.Instance==ADC1)
  {

	GPIO_InitTypeDef GPIO_InitStruct;

    /* ADC1 clock enable */
    //__HAL_RCC_ADC1_CLK_ENABLE();

	//GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HW_GPIO_Init(GPIOA, pin, &GPIO_InitStruct);
    //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void ADC_PinDeInit(uint16_t pin)
{

  if(hadc.Instance==ADC1)
  {

    /* Peripheral clock disable */
    //__HAL_RCC_ADC1_CLK_DISABLE();
	RCC_GPIO_CLK_DISABLE( (uint32_t) GPIOA);
    HAL_GPIO_DeInit(GPIOA, pin);

  }
}

uint16_t ADC_ReadChannel(uint32_t Channel)
{

  ADC_ChannelConfTypeDef adcConf;
  uint16_t adcData = 0;

 // if( AdcInitialized == true )
//  {
    /* wait the the Vrefint used by adc is set */
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VREFINTRDY) == RESET) {};


    __HAL_RCC_ADC1_CLK_ENABLE() ;

    /*calibrate ADC if any calibration hardware*/
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED );
    /* Deselects all channels*/
    adcConf.Channel = ADC_CHANNEL_MASK;
    adcConf.Rank = ADC_RANK_NONE;
    HAL_ADC_ConfigChannel(&hadc, &adcConf);

    /* configure adc channel */
    adcConf.Channel = Channel;
    adcConf.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc, &adcConf);
    /* Start the conversion process */
    HAL_ADC_Start( &hadc);
    /* Wait for the end of conversion */
    HAL_ADC_PollForConversion( &hadc, HAL_MAX_DELAY );

    /* Get the converted value of regular channel */
    adcData = HAL_ADC_GetValue ( &hadc);
    __HAL_ADC_DISABLE( &hadc) ;

    __HAL_RCC_ADC1_CLK_DISABLE() ;
 // }
  return adcData;
}

uint16_t getAnalogSensorValue(int ADC_InPin)
{
	//hadc = *p_hadc;
	uint16_t sensorValue = 0;
	/**ADC GPIO Configuration
	    ADC_IN0 ----> PA0 [or A0 or A1]
	    ADC_IN2 ----> PA2
	    ADC_IN3 ----> PA3
	    ADC_IN4 ----> PA4 (probably used on HW Battery level) [or A2 or A3]
	    ADC_IN5 ----> PA5
	*/
	uint16_t A_pin = -1;
	switch (ADC_InPin)
	{
		case 0: A_pin = GPIO_PIN_0;
		break;
		case 2: A_pin = GPIO_PIN_2;
		break;
		case 3: A_pin = GPIO_PIN_3;
		break;
		case 5: A_pin = GPIO_PIN_5;
		break;
		default: //debug
		break;
	}
	if(A_pin!=-1)
	{
		ADC_PinInit(A_pin);
		sensorValue = ADC_ReadChannel(ADC_CHANNEL_0);
		ADC_PinDeInit(A_pin);
	}
	return sensorValue;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
