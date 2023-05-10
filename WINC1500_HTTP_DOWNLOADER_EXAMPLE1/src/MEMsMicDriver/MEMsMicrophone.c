/**************************************************************************/ /**
 * @file      MEMsMicrophone.c
 * @brief     Driver for the MEMs microphone. At ADC. Read data from the microphone

 This driver will be written compatible to be run from RTOS thread, with non-blocking commands in mind.
 See https://learn.sparkfun.com/tutorials/mems-microphone-hookup-guide/all for more information
 * @author    Aurchie
 * @date      2023-04-26

 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "MEMsMicDriver/MEMsMicrophone.h"
#include "WifiHandlerThread/WifiHandler.h"

/******************************************************************************
 * Variables
 ******************************************************************************/

/******************************************************************************
 * Structures and Enumerations
 ******************************************************************************/

/******************************************************************************
 *  Callback Declaration
 ******************************************************************************/

/******************************************************************************
 * Local Function Declaration
 ******************************************************************************/
static void configure_adc(void);

/******************************************************************************
 * Global Local Variables
 ******************************************************************************/

/******************************************************************************
 * Global Functions
 ******************************************************************************/

/**
 * @fn			void mic_init(void)
 * @brief		Initialises the ADC module
 * @details		Initialises the ADC module
 * @note		Call from main once to initialise hardware.
 */
void mic_init(void)
{
    // Configure USART and Callbacks
    configure_adc();
}

/**
 * @fn			void configure_adc(void)
 * @brief		Configure the ADC module
 * @details		Configure the ADC module
 * @note		Call from microphone intialisation {mic_init()}
 *				once to configure the ADC instance.
 */
void configure_adc(void)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);
	#if (SAMC21)
	adc_init(&adc_instance, ADC1, &config_adc);
	#else
	adc_init(&adc_instance, ADC, &config_adc);
	#endif
	adc_enable(&adc_instance);
}

/**
 * @fn			uint16_t mic_read(void)
 * @brief		Read microphone data
 * @details		Read converted analog data from MEMs microphone
 * @note		Will be called from from CLI when the command is
 *				entered in console; command is 'mic'
 */
int32_t mic_read(void)
{
	adc_start_conversion(&adc_instance);
	uint16_t mic_data;
	do {
		/* Wait for conversion to be done and read out result */
	} while (adc_read(&adc_instance, &mic_data) == STATUS_BUSY);
	char buff[20];
	//snprintf(buff, 20, "Mic data: %d\r\n", mic_data);
	//SerialConsoleWriteString(buff);
	vTaskDelay(1000);
	return WifiAddMicDataToQueue(&mic_data);
}

