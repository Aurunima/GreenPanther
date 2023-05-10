/**************************************************************************/ /**
 * @file      MEMsMicrophone.c
 * @brief     Driver for the MEMs microphone. At ADC. Read data from the microphone

 This driver will be written compatible to be run from RTOS thread, with non-blocking commands in mind.
 See https://learn.sparkfun.com/tutorials/mems-microphone-hookup-guide/all for more information
 * @author    Aurchie
 * @date      2023-04-26

 ******************************************************************************/

#ifndef MEMS_MIC_H
#define MEMS_MIC_H

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <asf.h>
#include <stdarg.h>
#include "SerialConsole.h"
#include "string.h"

/******************************************************************************
 * Defines
 ******************************************************************************/

/******************************************************************************
 * Structures and Enumerations
 ******************************************************************************/
struct adc_module adc_instance;  ///< mic ADC module

/******************************************************************************
 * Global Function Declarations
 ******************************************************************************/

/******************************************************************************
 * Functions
 ******************************************************************************/
void mic_init(void);
int32_t mic_read(void);

#endif /*MEMS_MIC_H*/