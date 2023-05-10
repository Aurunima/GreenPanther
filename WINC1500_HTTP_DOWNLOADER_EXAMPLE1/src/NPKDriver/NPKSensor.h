/**************************************************************************/ /**
 * @file      NPKSensor.h
 * @brief     Driver for the Soil NPK Sensor, in SERIAL MODE. This means the jumper must remain on!
 * In this mode, it will work with an UART driver.
 * The MCU can send the following inquiry frame
 * - Nitrogen: {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c}
 * - Phosphorus: {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc}
 * - Potassium: {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0}
 This driver will be written compatible to be run from RTOS thread, with non-blocking commands in mind.
 See https://how2electronics.com/measure-soil-nutrient-using-arduino-soil-npk-sensor/ for more information
 * @author    Aurchie
 * @date      4th May 2023

 ******************************************************************************/

#ifndef NPK_SENSOR_H
#define NPK_SENSOR_H

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <asf.h>
#include <stdarg.h>

#include "string.h"

/******************************************************************************
 * Defines
 ******************************************************************************/
//#define NITROGEN {0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c}
	
/******************************************************************************
 * Structures and Enumerations
 ******************************************************************************/

/******************************************************************************
 * Global Function Declarations
 ******************************************************************************/

/******************************************************************************
 * Functions
 ******************************************************************************/
void InitialiseNPKSensor(void);
void DeinitialiseNPKSensor(void);

//int32_t NPKSensorGetNPK(uint16_t *NPK, const TickType_t xMaxBlockTime);
int32_t NPKSensorGetNitrogen(uint16_t *nitrogen, const TickType_t xMaxBlockTime);
int32_t NPKSensorGetPhosphorus(uint16_t *phosphorus, const TickType_t xMaxBlockTime);
int32_t NPKSensorGetPotassium(uint16_t *potassium, const TickType_t xMaxBlockTime);
void npkUsartWritecallback(struct usart_module *const usart_module);
void npkUsartReadcallback(struct usart_module *const usart_module);

#endif NPK_SENSOR_H
