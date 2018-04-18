/*
 * tsl2561.h
 *
 *  Copyright (C) 2018 Denis Ronchese <denis.ronchese@gmail.com>
 *  Created on: 16.04.2018
 *      Author: dk8moore
 */

#ifndef __TSL2561_H
#define __TSL2561_H

#include "stm32l0xx.h"
#include "stm32l0xx_hal.h" //changed from f0xx to l0xx to make it compatible with this board
#include <stdint.h>
#include <stdbool.h>


/**
 *  Physical addresses available for I2C communication
 */
#define TSL2561_I2C_ADDRESS_0 	0x29 // GND on ADDR SEL pin of the sensor
#define TSL2561_I2C_ADDRESS_1 	0x39 // FLOAT on ADDR SEL pin of the sensor
#define TSL2561_I2C_ADDRESS_2 	0x49 // VDD on ADDR SEL pin of the sensor

#define TSL2560_CHIP_ID		0x00 /* MSByte -> TSL2561 is actually TSL2560 */
#define TSL2561_CHIP_ID  	0x01 /* MSByte -> TSL2561 */


/**
 *  Integration time scaling factors
 */
#define CH_SCALE		10		// scale channel values by 2^10
#define CHSCALE_TINT0	0x7517	// 322/11 * 2^CH_SCALE
#define CHSCALE_TINT1	0x0fe7	// 322/81 * 2^CH_SCALE


/**
 *  Coefficients for the LUX calculations (p. 24/26 of datasheet)
 */
#define LUX_SCALE 	14			// 2^14
#define RATIO_SCALE 9			// 2^9=512
// -> FN, T and CL packages
#define K1T 		0x0040 		// 0.125 * 2^RATIO_SCALE
#define B1T 		0x01f2 		// 0.0304 * 2^LUX_SCALE
#define M1T 		0x01be 		// 0.0272 * 2^LUX_SCALE

#define K2T 		0x0080 		// 0.250 * 2^RATIO_SCALE
#define B2T 		0x0214 		// 0.0325 * 2^LUX_SCALE
#define M2T 		0x02d1		// 0.0440 * 2^LUX_SCALE

#define K3T 		0x00c0		// 0.375 * 2^RATIO_SCALE
#define B3T 		0x023f 		// 0.0351 * 2^LUX_SCALE
#define M3T 		0x037b 		// 0.0544 * 2^LUX_SCALE

#define K4T 		0x0100 		// 0.50 * 2^RATIO_SCALE
#define B4T 		0x0270 		// 0.0381 * 2^LUX_SCALE
#define M4T 		0x03fe 		// 0.0624 * 2^LUX_SCALE

#define K5T 		0x0138 		// 0.61 * 2^RATIO_SCALE
#define B5T 		0x016f 		// 0.0224 * 2^LUX_SCALE
#define M5T 		0x01fc 		// 0.0310 * 2^LUX_SCALE

#define K6T 		0x019a 		// 0.80 * 2^RATIO_SCALE
#define B6T 		0x00d2 		// 0.0128 * 2^LUX_SCALE
#define M6T 		0x00fb 		// 0.0153 * 2^LUX_SCALE

#define K7T 		0x029a 		// 1.3 * 2^RATIO_SCALE
#define B7T 		0x0018 		// 0.00146 * 2^LUX_SCALE
#define M7T 		0x0012 		// 0.00112 * 2^LUX_SCALE

#define K8T 		0x029a 		// 1.3 * 2^RATIO_SCALE
#define B8T 		0x0000 		// 0.000 * 2^LUX_SCALE
#define M8T 		0x0000 		// 0.000 * 2^LUX_SCALE
// -> CS package
#define K1C 		0x0043 		// 0.130 * 2^RATIO_SCALE
#define B1C 		0x0204 		// 0.0315 * 2^LUX_SCALE
#define M1C 		0x01ad 		// 0.0262 * 2^LUX_SCALE

#define K2C 		0x0085 		// 0.260 * 2^RATIO_SCALE
#define B2C 		0x0228 		// 0.0337 * 2^LUX_SCALE
#define M2C 		0x02c1		// 0.0430 * 2^LUX_SCALE

#define K3C 		0x00c8		// 0.390 * 2^RATIO_SCALE
#define B3C 		0x0253 		// 0.0363 * 2^LUX_SCALE
#define M3C 		0x0363 		// 0.0529 * 2^LUX_SCALE

#define K4C 		0x010a 		// 0.520 * 2^RATIO_SCALE
#define B4C 		0x0282 		// 0.0392 * 2^LUX_SCALE
#define M4C 		0x03df 		// 0.0605 * 2^LUX_SCALE

#define K5C 		0x014d 		// 0.65 * 2^RATIO_SCALE
#define B5C 		0x0177 		// 0.0229 * 2^LUX_SCALE
#define M5C 		0x01dd 		// 0.0291 * 2^LUX_SCALE

#define K6C 		0x019a 		// 0.80 * 2^RATIO_SCALE
#define B6C 		0x0101 		// 0.0157 * 2^LUX_SCALE
#define M6C 		0x0127 		// 0.0180 * 2^LUX_SCALE

#define K7C 		0x029a 		// 1.3 * 2^RATIO_SCALE
#define B7C 		0x0037 		// 0.00338 * 2^LUX_SCALE
#define M7C 		0x002b 		// 0.00260 * 2^LUX_SCALE

#define K8C 		0x029a 		// 1.3 * 2^RATIO_SCALE
#define B8C 		0x0000 		// 0.000 * 2^LUX_SCALE
#define M8C 		0x0000 		// 0.000 * 2^LUX_SCALE


/**
 *  Definition of the Handler of the TSL2561
 */
typedef struct {

    uint16_t addr;

    I2C_HandleTypeDef* i2c;

    bool state;			//state of power-on or power-off

    uint8_t  id;        /* Chip ID */

} TSL2561_HandleTypedef;


/**
 *  Public functions headers
 */
bool tsl2561_init(TSL2561_HandleTypedef *dev);
unsigned long tsl2561_read_intensity(TSL2561_HandleTypedef *dev);

#endif /* TSL2561_H_ */
