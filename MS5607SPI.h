/*
   MS5607-02 SPI library for ARM STM32F103xx Microcontrollers - Main source file
   05/01/2020 by Joao Pedro Vilas <joaopedrovbs@gmail.com>
   Changelog:
     2012-05-23 - initial release.
*/
/* ============================================================================================
 MS5607-02 device SPI library code for ARM STM32F103xx is placed under the MIT license
Copyright (c) 2020 João Pedro Vilas Boas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 ================================================================================================
 */

#ifndef _MS5607SPI_H_
#define _MS5607SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* MS5607 SPI COMMANDS */
#define RESET_COMMAND                 0x1E
#define PROM_READ(address)            (0xA0 | ((address) << 1))         // Macro to change values for the 8 PROM addresses
#define CONVERT_D1_COMMAND            0x40
#define CONVERT_D2_COMMAND            0x50
#define READ_ADC_COMMAND              0x00

/* MS5607 Oversampling Ratio Enumeration*/
typedef enum OSRFactors{
  OSR_256,
  OSR_512=0x02,
  OSR_1024=0x04,
  OSR_2048=0x06,
  OSR_4096=0x08
}MS5607OSRFactors;

/* MS5607 System States Enumeration*/
typedef enum MS5607States{
  MS5607_STATE_FAILED,
  MS5607_STATE_READY
}MS5607StateTypeDef;

/* MS5607 PROM Data Structure */
struct promData{
  uint16_t reserved;
  uint16_t sens;
  uint16_t off;
  uint16_t tcs;
  uint16_t tco;
  uint16_t tref;
  uint16_t tempsens;
  uint16_t crc;
};

/* Uncompensated digital Values */
struct MS5607UncompensatedValues{
  uint32_t  pressure;
  uint32_t  temperature;
};

/* Actual readings */
struct MS5607Readings{
  int32_t  pressure;
  int32_t  temperature;
};

/**
 * @brief  Initializes MS5607 Sensor
 * @param  SPI Handle address
 * @param  GPIO Port Definition
 * @param  GPIO Pin
 * @retval Initialization status:
 *           - 0 or MS5607_STATE_FAILED: Was not abe to communicate with sensor
 *           - 1 or MS5607_STATE_READY: Sensor initialized OK and ready to use
 */
MS5607StateTypeDef MS5607_Init(SPI_HandleTypeDef *, GPIO_TypeDef *, uint16_t);

/**
 * @brief  Reads MS5607 PROM Content
 * @note   Must be called only on device initialization
 * @param  Address of promData structure instance
 * @retval None
 */
void MS5607PromRead(struct promData *);

/**
 * @brief  Reads uncompensated content from the MS5607 ADC
 * @note   Must be called before every convertion
 * @param  Address of MS5607 UncompensatedValues Structure
 * @retval None
 */
void MS5607UncompensatedRead(struct MS5607UncompensatedValues *);

/**
 * @brief  Converts uncompensated values into real world values using data from @ref promData and @ref MS5607UncompensatedValues
 * @note   Must be called after @ref MS5607UncompensatedRead
 * @param  Address of MS5607UncompensatedValues Structure
 * @param  Address of MS5607Readings Structure
 * @retval None
 */
void MS5607Convert(struct MS5607UncompensatedValues *, struct MS5607Readings *);

/**
 * @brief  Updates the readings from the sensor
 * @note   This function must be called each time you want new values from the sensor
 * @param  None
 * @retval None
 */
void MS5607Update(void);

/**
 * @brief  Gets the temperature reading from the last sensor update
 * @note   This function must be called after an @ref MS5607Update()
 * @param  None
 * @retval Temperature in celsius
 */
double MS5607GetTemperatureC(void);

/**
 * @brief  Gets the pressure reading from the last sensor update
 * @note   This function must be called after an @ref MS5607Update()
 * @param  None
 * @retval Pressure in Pascal
 */
int32_t MS5607GetPressurePa(void);

/**
 * @brief  Enables the chip select pin
 * @param  None
 * @retval None
 */
void enableCSB(void);

/**
 * @brief  Disables the chip select pin
 * @param  None
 * @retval None
 */
void disableCSB(void);

/**
 * @brief  Sets the temperature reading oversamplig ratio
 * @param  OSR factor from 256 to 4096
 * @retval None
 */
void MS5607SetTemperatureOSR(MS5607OSRFactors);

/**
 * @brief  Sets the pressure reading oversamplig ratio
 * @param  OSR factor from 256 to 4096
 * @retval None
 */
void MS5607SetPressureOSR(MS5607OSRFactors);

#ifdef __cplusplus
}
#endif

#endif /* _MS5607SPI_H_ */
