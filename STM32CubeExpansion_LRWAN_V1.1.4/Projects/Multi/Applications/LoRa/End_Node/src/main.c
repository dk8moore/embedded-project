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
#include <string.h>
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
#include "delay.h"
#include "sx1276.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LPP_APP_PORT 99
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE 							LORAWAN_ADR_OFF // mod
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE 					DR_0 //DR_5 near transmission
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
#define LORAWAN_APP_DATA_BUFF_SIZE                  59 // max payload size allowed by DR_0

/*!
 * ID of the device on the server front applications
 */
#define TOKEN_DEVICE								776522  //Not useful, added on server

/*!
 * Number of Sensors readings before send
 */
#define READ_NUMBER									15 //15

/*!
 * Seconds on sleep
 */
#define SLEEP_DUTYCYCLE                             7000 //60



/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
static lora_AppData_t AppData = {AppDataBuff, 0, 0};


/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* LoRa endNode send request*/
static void Send(char message[LORAWAN_APP_DATA_BUFF_SIZE]);

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* Tx timer callback function */
static void OnTxTimerEvent(void);

/* Tx button callback function */
static void OnDemandEvent(void);

// --------------------------------

/* Procedure that call all the init functions for the 3 sensors connected to the board */
static void Init_Sensors(void);

/* Read a single value from all the sensors connected */
static bool Read_Sensors(void);

/* Sum the now reading to the past ones and increment the reading counter */
static void Sum_Readings(void);

/* Print on the vcom the values read by the sensors */
static void Print_Sensors(void);

/* Return the converted Bar value of the pressure in Pascal */
static float Pa_to_Bar(float pa);

/* Make the average of the readings btw two Lora sends */
static void Sensors_Average(float times, float avgs[5]);

/* Build the JSON string with the sensors data inside to send */
static void Build_JSON_Payload(float avgs[5], char payload[LORAWAN_APP_DATA_BUFF_SIZE]);

/* Make the copy of one array to the other */
static void AverageCopy(float avg_out[5], float avg_in[5]);

/* Infinite cycle that waits for the usrbutton to be pressed */
static void WaitUserButton(void);

/* Main sensor-read and data-send routine (callback to wake-up) */
static void Main_Routine(void);

/* Initialize the sleep timer (time specified by SLEEP_DUTYCYCLE) */
static void Init_Sleep_Timer(void);


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
static TimerEvent_t WakeTimer;

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};


BMP280_HandleTypedef bmp280;
TSL2561_HandleTypedef tsl2561;
bool bme280p, tsl2561p;

float temp, temp_sum;
float pres, pres_sum;
float hum, hum_sum;
u_int16_t gas, gas_sum;
unsigned long lux, lux_sum;
float past_avgs[5] = { -1.0, -1.0, -1.0, -1.0, -1.0 };

int read_counter;
bool ft_send;

char str_gas[15]; // string for output on st-link the gas sensor read-out
char str_bmp[30]; // string for output on st-link the environmental sensor read-out
char str_hum[15];
char str_lux[15]; // string for output on st-link the light sensor read-out
char env_sen[50]; // string for output on st-link the type of environmental sensor connected
char lux_sen[50]; // string for output on st-link the type of light sensor connected
char str_sen[50];


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	// SYSTEM INITIALIZATION

	ft_send = true;

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

	/* Initialise the sensors connected */
	Init_Sensors();

	/* Start a timer to transmit data on LORA network */
	//LoraStartTx(TX_ON_TIMER); //TX_ON_EVENT allows to start LoRa transmission by pressing the USER BUTTON
	Init_Sleep_Timer();

	while(1)
	{
		DISABLE_IRQ();
		if(LoRaTxDone==1)
		{
			LPM_EnterLowPower();
		} else DelayMs(10);
		ENABLE_IRQ();
		/*
		if(Read_Sensors())
		{
			Sum_Readings();
			//Print_Sensors();
		}
		else PRINTF("Error in reading!");
		*/
	}
}

static void WaitUserButton(void)
{
	/* Wait Until User push-button pressed */
	while(BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_RESET)
	{
	}
	/* Wait Until User push-button released */
	while(BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_SET)
	{
	}
}

static void Main_Routine(void)
{
	LoRaTxDone=0;
	read_counter=0;
	if(ft_send==false)
	{
		while(read_counter<READ_NUMBER)
		{
			if(Read_Sensors())
			{
				Sum_Readings();
				//Print_Sensors();
			}
			else PRINTF("Error in reading!");
		}
	}
	OnDemandEvent();
	TimerStart(&WakeTimer); //WakeTimer sarebbe da far ripartire in sx1276.h
}

static void Init_Sleep_Timer(void)
{
	TimerInit(&WakeTimer, Main_Routine);
	TimerSetValue(&WakeTimer, SLEEP_DUTYCYCLE);
	//TimerStart(&WakeTimer);
	Main_Routine();
}

static void Init_Sensors(void)
{
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
		PRINTF("TSL2561 initialization failed!\n\r");
	}

	/* Check ID of TSL2561 */
	tsl2561p = ((tsl2561.id && 0xF0) >> 4) == TSL2561_CHIP_ID; //0x01 value (TSL2561)
	sprintf(lux_sen, "Light sensor found: %s\n\r",tsl2561p ? "TSL2561" : "TSL2560");
	PRINTF(lux_sen);

	/* Initialize the reading variables */
	pres = 0.0;
	temp = 0.0;
	hum = 0.0;
	gas = 0;
	lux = 0;
	pres_sum = 0.0;
	temp_sum = 0.0;
	hum_sum = 0.0;
	gas_sum = 0;
	lux_sum = 0;
	read_counter = 0;



	return;
}

static bool Read_Sensors(void)
{
	gas = getAnalogSensorValue(5); //uses adc.h -> initialize Adc then read Adc value then de-init Adc (on pin ADC_IN5)
	// SENSORS READING OPERATIONS
	/* ENVIRONMENTAL SENSOR BMx280 READ OPERATIONS */
	HAL_Delay(100);
	if(!bmp280_read_float(&bmp280, &temp, &pres, &hum)) {  //uses bmp280.h
		PRINTF("BMx280 reading failed!\n\r");
		return false;
	}
	pres = Pa_to_Bar(pres);
	HAL_Delay(100);
	if(!tsl2561_read_intensity(&tsl2561, &lux)) {	//uses tsl2561.h
		PRINTF("TSL2561 reading failed!\n\r");
		return false;
	}
	HAL_Delay(100);
	return true;
}

static void Sum_Readings(void)
{
	temp_sum += temp;
	pres_sum += pres;
	hum_sum += hum;
	gas_sum += gas;
	lux_sum += lux;
	read_counter++;
}

static void Print_Sensors(void)
{
	sprintf(str_bmp,"T: %.1f °C, P: %.3f Pa, ", temp, pres); // NEED TO ADD LINE IN LINKER FLAGS -> -u _printf_float TO ENABLE FLOAT PRINT
	if (bme280p) {
		sprintf(str_hum,"H: %.2f %%%%, ", hum);
	}
	sprintf(str_lux,"L: %d lx\n\r", (int)lux);
	sprintf(str_gas,"G: %d ppm, ", (int)gas);
	PRINTF(str_bmp);
	PRINTF(str_hum);
	PRINTF(str_gas);
	PRINTF(str_lux);
}

static void Sensors_Average(float times, float avgs[5]) //input: times (number of readings), output: array of averages
{
	avgs[0] = roundf((temp_sum/times)*10)/10;
	avgs[1] = roundf((pres_sum/times)*1000)/1000;
	avgs[2] = roundf((hum_sum/times)*100)/100;
	avgs[3] = gas_sum/times;
	avgs[4] = lux_sum/times;

	//Print_Sensors_Average
	/*
	sprintf(str_bmp,"AVERAGE -> T: %.1f °C, p: %.3f bar, ", avgs[0], avgs[1]); // NEED TO ADD LINE IN LINKER FLAGS -> -u _printf_float TO ENABLE FLOAT PRINT
	if (bme280p) {
		sprintf(str_hum,"h: %.2f %%%%, ", avgs[2]);
	}
	sprintf(str_gas,"G: %d ppm, ", (int)avgs[3]);
	sprintf(str_lux,"L: %d lx\n\r", (int)avgs[4]);
	PRINTF(str_bmp);
	PRINTF(str_hum);
	PRINTF(str_gas);
	PRINTF(str_lux);
	*/
	return;
}

static float Pa_to_Bar(float pa)
{
	return pa/pow(10,5);
}

static void AverageCopy(float avg_out[5], float avg_in[5])
{
	for(int i=0;i<5;i++)
	{
		avg_out[i] = avg_in[i];
	}
	return;
}

static void Send(char message[LORAWAN_APP_DATA_BUFF_SIZE])
{
	/* Join the LORA network */
	LORA_Join();

	if(LORA_JoinStatus()!=LORA_SET)
	{
		PRINTF("Not Joined, try again later..\n\r");
		return;
	}
	HAL_GPIO_WritePin(RADIO_TCXO_VCC_PORT, RADIO_TCXO_VCC_PIN, GPIO_PIN_SET);
	AppData.Port = LORAWAN_APP_PORT;
	memcpy(AppData.Buff, message, strlen(message)+1);
	AppData.BuffSize = strlen(message)+1;
	LORA_send(&AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
}

static void Build_JSON_Payload(float avgs[5], char payload[LORAWAN_APP_DATA_BUFF_SIZE])
{
	//char head[1];
	//sprintf(head, "{\"D\":\"%d\"", TOKEN_DEVICE); //si può togliere aggiungendolo lato server -> risparmio 16
	char head = '{';
	char temp_pl[9] = ""; //16
	char pres_pl[10] = ""; //17
	char hum_pl[12] = ""; //17
	char gas_pl[11] = ""; //17
	char lux_pl[12] = ""; //18
	char tail = '}';

	// INCREMENTAL SENDING -> BETA -> a double send needs to be implemented
	/*
	if(past_avgs[0]!=avgs[0])
		sprintf(temp_pl, ",\"T\":\"%.1f\"", avgs[0]);
	if(past_avgs[1]!=avgs[1])
		sprintf(pres_pl, ",\"p\":\"%.3f\"", avgs[1]);
	if(past_avgs[2]!=avgs[2])
		sprintf(hum_pl, ",\"h\":\"%.2f\"", avgs[2]);
	if((int)past_avgs[3]!=(int)avgs[3])
		sprintf(gas_pl, ",\"g\":\"%d\"", (int)avgs[3]);
	if((int)past_avgs[4]!=(int)avgs[4])
		sprintf(lux_pl, ",\"l\":\"%d\"", (int)avgs[4]);

	AverageCopy(past_avgs, avgs);
	 */

	// FULL PAYLOAD ALWAYS
	/*
	sprintf(temp_pl, ",\"T\":\"%.1f\"", avgs[0]);
	sprintf(pres_pl, ",\"p\":\"%.3f\"", avgs[1]);
	sprintf(hum_pl, ",\"h\":\"%.2f\"", avgs[2]);
	sprintf(gas_pl, ",\"g\":\"%d\"", (int)avgs[3]);
	sprintf(lux_pl, ",\"l\":\"%d\"", (int)avgs[4]);
	*/
	// Tolgo l'invio dei dati in stringa -> risparmio 20
	sprintf(temp_pl, "\"T\":%d", (int)(avgs[0]*10));
	sprintf(pres_pl, ",\"p\":%d", (int)(avgs[1]*1000));
	sprintf(hum_pl, ",\"h\":%d", (int)(avgs[2]*100));
	sprintf(gas_pl, ",\"g\":%d", (int)avgs[3]);
	sprintf(lux_pl, ",\"l\":%d", (int)avgs[4]);

	sprintf(payload, "%c%s%s%s%s%s%c", head, temp_pl, pres_pl, hum_pl, gas_pl, lux_pl, tail); //TOTAL
}

static void OnDemandEvent(void)
{
	float values[5];
	char payload[LORAWAN_APP_DATA_BUFF_SIZE] = "";

	if(ft_send==true)
	{
		sprintf(payload, "{\"D\":%d}", TOKEN_DEVICE);
		ft_send = false;
	}
	else
	{
		Sensors_Average((float)read_counter, values);
		pres_sum = 0.0;
		temp_sum = 0.0;
		hum_sum = 0.0;
		gas_sum = 0;
		lux_sum = 0;
		read_counter = 0;
		Build_JSON_Payload(values, payload);
	}
	Send(payload);
	PRINTF(payload);
	PRINTF("\n\r");
}

static void OnTxTimerEvent(void)
{
	float values[5];
	char payload[LORAWAN_APP_DATA_BUFF_SIZE] = "";

	Sensors_Average((float)read_counter, values);
	pres_sum = 0.0;
	temp_sum = 0.0;
	hum_sum = 0.0;
	gas_sum = 0;
	lux_sum = 0;
	read_counter = 0;

	if(ft_send==true)
	{
		sprintf(payload, "{\"D\":\"%d\"}", TOKEN_DEVICE);
		ft_send = false;

	}
	else Build_JSON_Payload(values, payload);

	Send(payload);

	PRINTF(payload);
	PRINTF("\n\r");
	//Send("{\"D\":\"776522\",\"T\":\"20.1\",\"p\":\"0.975\",\"h\":\"100.00\",\"g\":\"111111\",\"l\":\"11111111\"}");
	//PRINTF("{\"T\":999,\"p\":999,\"h\":99999,\"g\":9999,\"l\":99999}"); //max must be 59

	/*Wait for next tx slot*/
	TimerStart(&TxTimer);

}

// --------------------------------------------------------------------

static void LoraStartTx(TxEventType_t EventType)
{
	if (EventType == TX_ON_TIMER)
	{
		 /* send everytime timer elapses */
		 TimerInit(&TxTimer, OnTxTimerEvent);
		 TimerSetValue(&TxTimer, APP_TX_DUTYCYCLE);
		 OnTxTimerEvent();
	}
	else
	{
		/* send everytime button is pushed (NEED MODIFICATIONS) */
		GPIO_InitTypeDef initStruct={0};
		initStruct.Mode = GPIO_MODE_IT_RISING;
		initStruct.Pull = GPIO_PULLUP;
		initStruct.Speed = GPIO_SPEED_HIGH;
		HW_GPIO_Init(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct);
		HW_GPIO_SetIrq(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, OnDemandEvent);
	}
}

static void LORA_HasJoined( void )
{
	#if( OVER_THE_AIR_ACTIVATION != 0 )
		PRINTF("JOINED\n\r");
	#endif
	LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
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
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
