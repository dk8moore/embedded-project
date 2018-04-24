/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "adc.h"
#include "stm32l0xx_hal.h"
#include "i2c.h"
#include "bmp280.h"
#include "tsl2561.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*!
 * CAYENNE_LPP is myDevices Application server.
 */
//#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_APP_PORT 99
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            4000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE 							LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE 					DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            3
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                  64

/*!
 * User application data buffer size
 */
#define TOKEN_DEVICE								7765214



/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* LoRa endNode send request*/
static void Send( void );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent( void );

/* Print on the vcom the values read by the sensors */
static void Print_Sensors(void);

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetTemperatureLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LORA_RxData,
                                               LORA_HasJoined,
                                               LORA_ConfirmClass};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;
                                               
static TimerEvent_t TxTimer;

#ifdef USE_B_L072Z_LRWAN1
	/*!
	 * Timer to handle the application Tx Led to toggle
	 */
	static TimerEvent_t TxLedTimer;
	static void OnTimerLedEvent( void );
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};


BMP280_HandleTypedef bmp280;
TSL2561_HandleTypedef tsl2561;
u_int16_t gas;
float pres, temp, hum;
unsigned long lux;
char str_gas[15]; // string for output on st-link the gas sensor read-out
char str_bmp[30]; // string for output on st-link the enviromental sensor read-out
char str_hum[15];
char str_lux[15]; // string for output on st-link the light sensor read-out
char env_sen[50]; // string for output on st-link the type of environmental sensor connected
char lux_sen[50]; // string for output on st-link the type of light sensor connected
char str_sen[50];
bool bme280p, tsl2561p;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	// SYSTEM INITIALIZATION

	/* STM32 HAL library initialisation*/
	HAL_Init();

	/* Configure the system clock*/
	SystemClock_Config();

	/* Configure the debug mode*/
	DBG_Init();

	/* Configure the hardware*/
	HW_Init();

	/*Disable Stand-by mode*/
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

	/* Configure the Lora Stack*/
	LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);
	PRINTF("VERSION: %X\n\r", VERSION);

	/* Join the LORA network */
	LORA_Join();

	/* Start a timer to trasmit data on LORA network */
	LoraStartTx(TX_ON_TIMER); //TX_ON_EVENT allows to start LoRa transmission by pressing the USER BUTTON

	pres = 0.0;
	temp = 0.0;
	hum = 0.0;
	gas = 0;
	lux = 0;

	//BMP280 INITIALIZATION

	/* I2C1 bus initialization */
	MX_I2C1_Init();

	/* BMP280 I2C initialization */
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0; //0x76 address (BME280 SDO pin at GND)
	bmp280.i2c = &hi2c1;
	while (!bmp280_init(&bmp280, &bmp280.params)) {
		PRINTF("BMP280 initialization failed!\n\r");
	}

	/* Check ID of BMP280 */
	bme280p = bmp280.id == BME280_CHIP_ID; //0x60 value (BME280, temperature+pressure+humidity)
	sprintf(env_sen, "Enviromental sensor found: %s\n\r",bme280p ? "BME280" : "BMP280");
	PRINTF(env_sen);

	//TSL2561 INITIALIZATION

	/* TSL2561 I2C initialization */
	tsl2561.addr = TSL2561_I2C_ADDRESS_1; //0x39 address (TSL251 ADDR SEL pin left FLOATING)
	tsl2561.i2c = &hi2c1;
	while (!tsl2561_init(&tsl2561)) {
		//PRINTF("TSL2561 initialization failed!\n\r");
	}

	/* Check ID of TSL2561 */
	tsl2561p = ((tsl2561.id && 0xF0) >> 4) == TSL2561_CHIP_ID; //0x01 value (TSL2561)
	//char tsl_chip[10];
	//sprintf(tsl_chip, "%02x\n\r", tsl2561.id);
	//PRINTF(tsl_chip);
	sprintf(lux_sen, "Light sensor found: %s\n\r",tsl2561p ? "TSL2561" : "TSL2560");
	PRINTF(lux_sen);

	while(1)
	{
		// SENSORS READING OPERATIONS
		/* ENVIRONMENTAL SENSOR BMx280 READ OPERATIONS */
		if(!bmp280_read_float(&bmp280, &temp, &pres, &hum)) {  //uses bmp280.h
			PRINTF("BMx280 reading failed!\n\r");
		}
		gas = getAnalogSensorValue(0); //uses adc.h -> initialize Adc then read Adc value then de-init Adc (on pin ADC_IN0)
		if(!tsl2561_read_intensity(&tsl2561, &lux)) {	//uses tsl2561.h
			PRINTF("TSL2561 reading failed!\n\r");
		}

		Print_Sensors();

	}
}

static void LORA_HasJoined( void )
{
	#if( OVER_THE_AIR_ACTIVATION != 0 )
		PRINTF("JOINED\n\r");
	#endif
	LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
}

static void Print_Sensors(void)
{
	sprintf(str_bmp,"T: %.2f °C, P: %.2f Pa, ", temp, pres); // NEED TO ADD LINE IN LINKER FLAGS -> -u _printf_float TO ENABLE FLOAT PRINT
	if (bme280p) {
		sprintf(str_hum,"H: %.2f %%%%, ", hum);
	}
	sprintf(str_lux,"L: %d lx\n\r", (int)lux);
	sprintf(str_gas,"G: %d ppm, ", (int)gas);
	PRINTF(str_bmp);
	PRINTF(str_hum);
	PRINTF(str_gas);
	PRINTF(str_lux);
	return;
}

static void Send( void )
{

	//uint8_t batteryLevel;
	uint16_t token = TOKEN_DEVICE;
	char message[85]; //43 "string" characters + 7 token + 6 temperature + 10 pressure + 6 humidity + 6 gas + 7 lux = 85

	if ( LORA_JoinStatus () != LORA_SET)
	{
		/*Not joined, try again later*/
		return;
	}

	PRINTF("SEND REQUEST\n\r");

	#ifdef USE_B_L072Z_LRWAN1
		TimerInit( &TxLedTimer, OnTimerLedEvent );

		TimerSetValue(  &TxLedTimer, 200);

		LED_On( LED_RED1 ) ;

		TimerStart( &TxLedTimer );
	#endif

	//uint32_t i = 0;

	//batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

	/*
	char head[] = "{\"D:\"";
	char tail[] = "}";
	char h_temp[] = ",\"T:\"";
	char h_pres[] = ",\"p:\"";
	char h_hum[] = ",\"h:\"";
	char h_gas[] = ",\"g:\"";
	char h_lux[] = ",\"L:\"";
	char m_temp[];
	char m_pres[];
	char m_hum[];
	char m_gas[];
	char m_lux[];*/

	AppData.Port = LORAWAN_APP_PORT;
	sprintf(message,"{\"deveui\":\"%d,\"T:\"%.2f,\"p:\"%.2f,\"h:\"%.2f,\"g:\"%d,\"L:\"%d}", (int)token, temp, pres, hum, (int)gas, (int)lux);
	memcpy(AppData.Buff, message, strlen(message)+1);
	AppData.BuffSize = strlen(message)+1;

	/*
	AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
	AppData.Buff[i++] = pressure & 0xFF;
	AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
	AppData.Buff[i++] = temperature & 0xFF;
	AppData.Buff[i++] = ( humidity >> 8 ) & 0xFF;
	AppData.Buff[i++] = humidity & 0xFF;
	AppData.Buff[i++] = batteryLevel;*/

	LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);

}


static void LORA_RxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
  DBG_PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case 3:
    /*this port switches the class*/
    if( AppData->BuffSize == 1 )
    {
      switch (  AppData->Buff[0] )
      {
        case 0:
        {
          LORA_RequestClass(CLASS_A);
          break;
        }
        case 1:
        {
          LORA_RequestClass(CLASS_B);
          break;
        }
        case 2:
        {
          LORA_RequestClass(CLASS_C);
          break;
        }
        default:
          break;
      }
    }
    break;
    case LORAWAN_APP_PORT:
    if( AppData->BuffSize == 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On( LED_BLUE ) ; 
      }
    }
    break;
  case LPP_APP_PORT:
  {
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
    if ( AppLedStateOn == RESET )
    {
      PRINTF("LED OFF\n\r");
      LED_Off( LED_BLUE ) ; 
      
    }
    else
    {
      PRINTF("LED ON\n\r");
      LED_On( LED_BLUE ) ; 
    }
    break;
  }
  default:
    break;
  }
  /* USER CODE END 4 */
}

static void OnTxTimerEvent( void )
{
	Send();
	/*Wait for next tx slot*/
	TimerStart(&TxTimer);
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent();
  }
  else
  {
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct={0};
  
    initStruct.Mode =GPIO_MODE_IT_RISING;
    initStruct.Pull = GPIO_PULLUP;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
    HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optional*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}


#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void )
{
	LED_Off(LED_RED1);
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
