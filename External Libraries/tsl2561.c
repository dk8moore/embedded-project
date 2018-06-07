/*
 * tsl2561.c
 *
 *  Copyright (C) 2018 Denis Ronchese <denis.ronchese@gmail.com>
 *  Created on: 16.04.2018
 *      Author: dk8moore
 */

#include "tsl2561.h"
#include <stdio.h>
#include "vcom.h"

/**
 * TSL2561 registers: for all registers is specified the Command 0x8- or 1000 bit only
 */
#define TSL2561_CTRL 				0x80	//control register
#define TSL2561_TMNG 				0x81	//timing register
#define THRESH_LOW_LOW 				0x82	//interrupt registers
#define THRESH_LOW_HIGH 			0x83
#define THRESH_HIGH_LOW 			0x84
#define THRESH_HIGH_HIGH 			0x85
#define INTER_CTRL				 	0x86	//interrupt control register
#define TSL2561_REG_ID 				0x8A
#define CHANNEL0_LOW_BYTE_ADDRESS 	0x8C	// 1000 1100 - CommandBit and Address of CH0 Data lower Byte Register. Read data low of channel0 Register 0xC with byte protocol
#define CHANNEL0_HIGH_BYTE_ADDRESS 	0x8D	// 1000 1101 - CommandBit and Address of CH0 Data upper Byte Register. Read data high of channel0 Register 0xD with byte protocol
#define CHANNEL1_LOW_BYTE_ADDRESS 	0x8E
#define CHANNEL1_HIGH_BYTE_ADDRESS 	0x8F

#define TSL2561_POWER_ON			0x03
#define TSL2561_POWER_OFF			0x00


/**
 *
 */
static bool read_register8(TSL2561_HandleTypedef *dev, uint8_t addr, uint8_t *value) {
	uint16_t tx_buff;
	uint8_t rx_buff = 0;
	tx_buff = (dev->addr << 1);
	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, &rx_buff, 1, 5000) == HAL_OK) {
		*value = rx_buff;
		return true;
	}
	else return false;

}

/**
 *
 */
static bool write_register8(TSL2561_HandleTypedef *dev, uint8_t addr, uint8_t value) {
	uint16_t tx_buff;
	tx_buff = (dev->addr << 1);
	if (HAL_I2C_Mem_Write(dev->i2c, tx_buff, addr, 1, &value, 1, 5000) == HAL_OK)
		return true;
	else return false;
}

/**
 *
 */
static inline bool read_data(TSL2561_HandleTypedef *dev, uint8_t addr, uint8_t *value, uint8_t len) {
	uint16_t tx_buff;
	tx_buff = (dev->addr << 1);
	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, value, len, 5000) == HAL_OK)
		return true;
	else return false;

}

/**
 *
 */
static bool read_ADC_channel(TSL2561_HandleTypedef *dev, uint8_t low_byte_addr, uint8_t high_byte_addr, uint16_t *data) {

	uint8_t received_data[2];
	received_data[0] = 0;
	received_data[1] = 0;
	*data = 0;

	if(!read_data(dev, low_byte_addr, &received_data[0], 1))
		return false;
	if(!read_data(dev, high_byte_addr, &received_data[1], 1))
		return false;

	*data = (uint16_t) received_data[0];
	*data |= (uint16_t) (received_data[1] << 8);
	return true;
}

/**
 *
 *	Arguments:	unsigned int gain: 		gain scaling, where 0->1x, 1->16x
 *				unsigned int i_time: 	integration time, where 0->13.7ms, 1->100ms, other->402ms
 *				unsigned long ch0: 		raw channel value from channel 0 of TSL256x
 *				unsigned long ch1: 		raw channel value from channel 1 of TSL256x
 *				int pack:				package type, 0->FN-T-CL or 1->CS
 *
 *
 */
static unsigned long lux_calculation(unsigned int gain, unsigned int i_time, unsigned long ch0, unsigned long ch1, int pack) {

	unsigned long ch_scale; //scaling factor to be applied on the channels values; it takes into account the gain and the i_time
	unsigned long channel0;
	unsigned long channel1;

	switch(i_time)
	{
		case 0: // 13.7ms
			ch_scale = CHSCALE_TINT0;
			break;
		case 1: // 101ms
			ch_scale = CHSCALE_TINT1;
			break;
		default: // 402ms -> no scaling
			ch_scale = (1 << CH_SCALE);
			break;
	}

	if(!gain) ch_scale = ch_scale << 4;  // scale 1x to 16x

	// scaling the channel values
	channel0 = (ch0 * ch_scale) >> CH_SCALE;
	channel1 = (ch1 * ch_scale) >> CH_SCALE;

	// Find the ratio of the channel values scaled to 512, protecting against the null-division
	unsigned long ratio_temp = 0;
	if(channel0!=0)
		ratio_temp = (channel1 << (RATIO_SCALE+1))/channel0;
	unsigned long ratio = (ratio_temp + 1) >> 1; // Rounding the ratio to its final value

	// Choosing coefficients from Ratio value
	unsigned int b, m;
	switch(pack)
	{
		case 0: // T, FN and CL package
			if ((ratio >= 0) && (ratio <= K1T))
				{ b = B1T; m = M1T; }
			else if (ratio <= K2T)
				{ b = B2T; m = M2T; }
			else if (ratio <= K3T)
				{ b = B3T; m = M3T; }
			else if (ratio <= K4T)
				{ b = B4T; m = M4T; }
			else if (ratio <= K5T)
				{ b = B5T; m = M5T; }
			else if (ratio <= K6T)
				{ b = B6T; m = M6T; }
			else if (ratio <= K7T)
				{ b = B7T; m = M7T; }
			else if (ratio > K8T)
				{ b = B8T; m = M8T; }
			break;
		case 1: // CS package
			if ((ratio >= 0) && (ratio <= K1C))
				{ b = B1C; m = M1C; }
			else if (ratio <= K2C)
				{ b = B2C; m = M2C; }
			else if (ratio <= K3C)
				{ b = B3C; m = M3C; }
			else if (ratio <= K4C)
				{ b = B4C; m = M4C; }
			else if (ratio <= K5C)
				{ b = B5C; m = M5C; }
			else if (ratio <= K6C)
				{ b = B6C; m = M6C; }
			else if (ratio <= K7C)
				{ b = B7C; m = M7C; }
			else if (ratio > K8C)
				{ b = B8C; m = M8C; }
			break;
	}

	// Actual lux value calculation from coefficients chosen earlier
	unsigned long temp = (channel0 * b) - (channel1 * m);

	// Don't allow negative lux values
	if(temp < 0)
		temp = 0;

	// Round the LSB (2^LUX_SCALE-1)
	temp += (1 << (LUX_SCALE-1));

	// Strip off fractional portion
	unsigned long lux = temp >> LUX_SCALE;

	return lux;
}

/**
 *
 */
bool tsl2561_init(TSL2561_HandleTypedef *dev)
{
	//char ctrl[15];
	if (dev->addr != TSL2561_I2C_ADDRESS_0 && dev->addr != TSL2561_I2C_ADDRESS_1 && dev->addr != TSL2561_I2C_ADDRESS_2) {
		return false;
	}

	if (!write_register8(dev, TSL2561_CTRL, TSL2561_POWER_ON)) {
		return false;
	}

	HAL_Delay(50);

	if (!read_register8(dev, TSL2561_REG_ID, &dev->id)) {
		return false;
	}

	//debugging the initialization -> powering on the device
	/*
	uint8_t value;
	if (!read_register8(dev, TSL2561_CTRL, &value)) {
		return false;
	}
	if(value == TSL2561_POWER_ON || value == 0x33 || value == 0x13)
		PRINTF("Device is on!\n\r");
	else {
		PRINTF("Device is off but should be on!\n\r");
		sprintf(ctrl, "%02x\n\r",value);
		PRINTF(ctrl);
		return false;
	}
	*/
	//end debug

	if (!write_register8(dev, TSL2561_CTRL, TSL2561_POWER_OFF)) {
		return false;
	}

	//debugging the initialization -> powering off the device
	/*
	if (!read_register8(dev, TSL2561_CTRL, &value)) {
			return false;
	}
	if(value == TSL2561_POWER_OFF || value == 0x30 || value == 0x10)
		PRINTF("Device is off!\n\r");
	else {
		PRINTF("Device is on but should be off!\n\r");
		sprintf(ctrl, "%02x\n\r",value);
		PRINTF(ctrl);
		return false;
	}*/
	//end debug

	return true;
}


/*
 * There are the two separate channels on the sensor, corresponding to two photodiodes that detect the light.
 * Ch1 is associated to the diode that detects only the infrared light spectrum while ch0 detects both the visible and the infrared light spectrum.
 * So, in order to calibrate the sensor within the human visible light spectrum range, ch1 will be subtracted from ch0.
 * */
bool tsl2561_read_intensity(TSL2561_HandleTypedef *dev, unsigned long *lux) {
	uint16_t ch0 = 0;
	uint16_t ch1 = 0;

	// Reading process of the ADC registers
	if(!write_register8(dev, TSL2561_CTRL, TSL2561_POWER_ON)) // power on the device
		return false;
	HAL_Delay(500);
	if(!read_ADC_channel(dev, CHANNEL0_LOW_BYTE_ADDRESS, CHANNEL0_HIGH_BYTE_ADDRESS, &ch0))
	{
		PRINTF("TSL2561: Channel0 unable to read!\n\r");
		return false;
	}
	if(!read_ADC_channel(dev, CHANNEL1_LOW_BYTE_ADDRESS, CHANNEL1_HIGH_BYTE_ADDRESS, &ch1))
	{
		PRINTF("TSL2561: Channel1 unable to read!\n\r");
		return false;
	}
	if(!write_register8(dev, TSL2561_CTRL, TSL2561_POWER_OFF)) // power off the device
		return false;

	*lux = lux_calculation(1,2,ch0,ch1,0);
	return true;
}
