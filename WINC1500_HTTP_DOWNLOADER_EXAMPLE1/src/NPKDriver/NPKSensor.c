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

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "NPKDriver/NPKSensor.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "I2cDriver/I2cDriver.h"
#include "SerialConsole/SerialConsole.h"

/******************************************************************************
 * Variables
 ******************************************************************************/

struct usart_module usart_instance_npk;  ///< NPK sensor UART module

SemaphoreHandle_t sensorNPKMutexHandle;      ///< Mutex to handle the sensor I2C bus thread access.
SemaphoreHandle_t sensorNPKSemaphoreHandle;  ///< Binary semaphore to notify task that we have received an I2C interrupt on the Sensor bus

/******************************************************************************
 * Structures and Enumerations
 ******************************************************************************/
uint8_t NITROGEN[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};	///< Command to send to the NPK sensor to request nitrogen value
uint8_t PHOSPHORUS[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};	///< Command to send to the NPK sensor to request phosphorus value
uint8_t POTASSIUM[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};	///< Command to send to the NPK sensor to request potassium value
uint8_t npkTx[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
//uint8_t latestRxnpk = 0;
uint8_t latestRxnpk[7];
uint8_t tmp = 0;
/******************************************************************************
 *  Callback Declaration
 ******************************************************************************/
// Callback for when we finish writing characters to UART
void npkUsartWritecallback(struct usart_module *const usart_module)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(sensorNPKSemaphoreHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
// Callback for when we finish writing characters to UART

void npkUsartReadcallback(struct usart_module *const usart_module)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(sensorNPKSemaphoreHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
 * Local Function Declaration
 ******************************************************************************/
static void configure_usart(void);
static void configure_usart_callbacks(void);
static int32_t NPKSensorFreeMutex(void);
static int32_t NPKSensorGetMutex(TickType_t waitTime);
/******************************************************************************
 * Global Local Variables
 ******************************************************************************/

/******************************************************************************
 * Global Functions
 ******************************************************************************/

/**
 * @fn			void InitialiseNPKSensor(void)
 * @brief		Initializes the UART - sets up the SERCOM to act as UART and registers the callbacks for
 *				asynchronous reads and writes.
 * @details		Initializes the UART - sets up the SERCOM to act as UART and registers the callbacks for
 *				asynchronous reads and writes.
 * @note			Call from main once to initialize Hardware.
 */

void InitialiseNPKSensor(void)
{
    // Configure USART and Callbacks
    configure_usart();
    configure_usart_callbacks();

    sensorNPKMutexHandle = xSemaphoreCreateMutex();
    sensorNPKSemaphoreHandle = xSemaphoreCreateBinary();

    if (NULL == sensorNPKMutexHandle || NULL == sensorNPKSemaphoreHandle) {
        SerialConsoleWriteString((char *)"Could not initialise soil NPK Sensor!");
    }
}

/**
 * @fn			void DeinitialiseNPKSensor(void)
 * @brief		Deinitialises the UART
 * @note
 */
void DeinitialiseNPKSensor(void)
{
    usart_disable(&usart_instance_npk);
}

/**
 * @fn			int32_t NPKSensorGetNitrogen (uint16_t *nitrogen, const TickType_t xMaxBlockTime)
 * @brief		Gets the NPK data from the soil NPK sensor.
 * @note			Returns 0 if successful. -1 if an error occurred
 */
int32_t NPKSensorGetNitrogen(uint16_t *nitrogen, const TickType_t xMaxBlockTime)
{
    int error = ERROR_NONE;
	char buf[64];

    // 1. Get MUTEX. NPKSensorGetMutex. If we cant get it, goto
    error = NPKSensorGetMutex(WAIT_I2C_LINE_MS);
    if (ERROR_NONE != error) goto exitf;
	snprintf((char *) buf, sizeof(buf), "After mutex: %d\r\n",error);
	SerialConsoleWriteString(buf);

    //---2. Initiate sending data. First populate TX with the nitrogen command. Use usart_write_buffer_job to transmit 1 character
    for (uint8_t i = 0; i < 8; i++) {
	    npkTx[i] = NITROGEN[i];
    }
	//npkTx[8] = NITROGEN;
    if (STATUS_OK != usart_write_buffer_job(&usart_instance_npk, (uint8_t *)&npkTx, 8)) {
        goto exitf;
    }

    // 3. )Wait until the TX finished. TX should release the binary semaphore - so wait until semaphore
    if (xSemaphoreTake(sensorNPKSemaphoreHandle, xMaxBlockTime) == pdTRUE) {
        /* The transmission ended as expected. We now delay until the I2C sensor is finished */

    } else {
        /* The call to ulTaskNotifyTake() timed out. */
        error = ERR_TIMEOUT;
        goto exitf;
    }
	snprintf((char *) buf, sizeof(buf), "After UART TX: %d\r\n",error);
	SerialConsoleWriteString(buf);

    // 4. Initiate an rx job - usart_read_buffer_job - to read two characters. Read into variable latestRxnpk
    usart_read_buffer_job(&usart_instance_npk, (uint8_t *)&latestRxnpk, 7);  // Kicks off constant reading of characters
	/*for (int i = 0; i<7; i++)
	{
		usart_read_buffer_job(&usart_instance_npk, (uint8_t *)&latestRxnpk, 1);  // Kicks off constant reading of characters
		tmp = (i==4)?latestRxnpk:tmp;
	}*/


    //---7. Wait for notification
    if (xSemaphoreTake(sensorNPKSemaphoreHandle, xMaxBlockTime) == pdTRUE) {
        /* The transmission ended as expected. We now delay until the I2C sensor is finished */
        //*nitrogen = (latestRxnpk[0] << 8) + latestRxnpk[1];
		//*nitrogen = tmp;
		*nitrogen = latestRxnpk[4];
    } else {
        /* The call to ulTaskNotifyTake() timed out. */
        //error = ERR_TIMEOUT;
        //goto exitf;
		*nitrogen = 0;
    }
	snprintf((char *) buf, sizeof(buf), "After UART RX: %d\r\n",error);
	SerialConsoleWriteString(buf);

exitf:
    // Release mutex and return error
    NPKSensorFreeMutex();

    return error;
}

int32_t NPKSensorGetPhosphorus(uint16_t *phosphorus, const TickType_t xMaxBlockTime)
{
    int error = ERROR_NONE;
	char buf[64];

    // 1. Get MUTEX. NPKSensorGetMutex. If we cant get it, goto
    error = NPKSensorGetMutex(WAIT_I2C_LINE_MS);
    if (ERROR_NONE != error) goto exitf;
	snprintf((char *) buf, sizeof(buf), "After mutex: %d\r\n",error);
	SerialConsoleWriteString(buf);

    //---2. Initiate sending data. First populate TX with the phosphorus command. Use usart_write_buffer_job to transmit 1 character
    for (uint8_t i = 0; i < 8; i++) {
	    npkTx[i] = POTASSIUM[i];
    }
    if (STATUS_OK != usart_write_buffer_job(&usart_instance_npk, (uint8_t *)&npkTx, 8)) {
        goto exitf;
    }

    // 3. )Wait until the TX finished. TX should release the binary semaphore - so wait until semaphore
    if (xSemaphoreTake(sensorNPKSemaphoreHandle, xMaxBlockTime) == pdTRUE) {
        /* The transmission ended as expected. We now delay until the I2C sensor is finished */

    } else {
        /* The call to ulTaskNotifyTake() timed out. */
        error = ERR_TIMEOUT;
        goto exitf;
    }
	snprintf((char *) buf, sizeof(buf), "After UART TX: %d\r\n",error);
	SerialConsoleWriteString(buf);

    // 4. Initiate an rx job - usart_read_buffer_job - to read two characters. Read into variable latestRxnpk
    usart_read_buffer_job(&usart_instance_npk, (uint8_t *)&latestRxnpk, 7);  // Kicks off constant reading of characters
	/*for (int i = 0; i<7; i++)
	{
		usart_read_buffer_job(&usart_instance_npk, (uint8_t *)&latestRxnpk, 1);  // Kicks off constant reading of characters
		tmp = (i==4)?latestRxnpk:tmp;
	}*/


    //---7. Wait for notification
    if (xSemaphoreTake(sensorNPKSemaphoreHandle, xMaxBlockTime) == pdTRUE) {
        /* The transmission ended as expected. We now delay until the I2C sensor is finished */
		*phosphorus = latestRxnpk[4];
    } else {
        /* The call to ulTaskNotifyTake() timed out. */
        //error = ERR_TIMEOUT;
        //goto exitf;
		*phosphorus = 0;
    }
	snprintf((char *) buf, sizeof(buf), "After UART RX: %d\r\n",error);
	SerialConsoleWriteString(buf);

exitf:
    // Release mutex and return error
    NPKSensorFreeMutex();

    return error;
}

/**
 * @fn			int32_t NPKSensorGetPotassium (uint16_t *potassium, const TickType_t xMaxBlockTime)
 * @brief		Gets the NPK data from the soil NPK sensor.
 * @note			Returns 0 if successful. -1 if an error occurred
 */
int32_t NPKSensorGetPotassium(uint16_t *potassium, const TickType_t xMaxBlockTime)
{
    int error = ERROR_NONE;
	char buf[64];

    // 1. Get MUTEX. NPKSensorGetMutex. If we cant get it, goto
    error = NPKSensorGetMutex(WAIT_I2C_LINE_MS);
    if (ERROR_NONE != error) goto exitf;
	snprintf((char *) buf, sizeof(buf), "After mutex: %d\r\n",error);
	SerialConsoleWriteString(buf);

    //---2. Initiate sending data. First populate TX with the potassium command. Use usart_write_buffer_job to transmit 1 character
    for (uint8_t i = 0; i < 8; i++) {
	    npkTx[i] = POTASSIUM[i];
    }
    if (STATUS_OK != usart_write_buffer_job(&usart_instance_npk, (uint8_t *)&npkTx, 8)) {
        goto exitf;
    }

    // 3. )Wait until the TX finished. TX should release the binary semaphore - so wait until semaphore
    if (xSemaphoreTake(sensorNPKSemaphoreHandle, xMaxBlockTime) == pdTRUE) {
        /* The transmission ended as expected. We now delay until the I2C sensor is finished */

    } else {
        /* The call to ulTaskNotifyTake() timed out. */
        error = ERR_TIMEOUT;
        goto exitf;
    }
	snprintf((char *) buf, sizeof(buf), "After UART TX: %d\r\n",error);
	SerialConsoleWriteString(buf);

    // 4. Initiate an rx job - usart_read_buffer_job - to read two characters. Read into variable latestRxnpk
    usart_read_buffer_job(&usart_instance_npk, (uint8_t *)&latestRxnpk, 7);  // Kicks off constant reading of characters
	/*for (int i = 0; i<7; i++)
	{
		usart_read_buffer_job(&usart_instance_npk, (uint8_t *)&latestRxnpk, 1);  // Kicks off constant reading of characters
		tmp = (i==4)?latestRxnpk:tmp;
	}*/


    //---7. Wait for notification
    if (xSemaphoreTake(sensorNPKSemaphoreHandle, xMaxBlockTime) == pdTRUE) {
        /* The transmission ended as expected. We now delay until the I2C sensor is finished */
		*potassium = latestRxnpk[4];
    } else {
        /* The call to ulTaskNotifyTake() timed out. */
        //error = ERR_TIMEOUT;
        //goto exitf;
		*potassium = 0;
    }
	snprintf((char *) buf, sizeof(buf), "After UART RX: %d\r\n",error);
	SerialConsoleWriteString(buf);

exitf:
    // Release mutex and return error
    NPKSensorFreeMutex();

    return error;
}

/**
 * @fn			static void configure_usart(void)
 * @brief		Code to configure the SERCOM "EDBG_CDC_MODULE" to be a UART channel running at 115200 8N1
 * @note
 */
static void configure_usart(void)
{
    struct usart_config config_usart;
    usart_get_config_defaults(&config_usart);

    config_usart.baudrate = 9600;
    config_usart.mux_setting = USART_RX_1_TX_0_XCK_1;
    config_usart.pinmux_pad0 = PINMUX_PB02D_SERCOM5_PAD0;
    config_usart.pinmux_pad1 = PINMUX_PB03D_SERCOM5_PAD1;
    config_usart.pinmux_pad2 = PINMUX_UNUSED;
    config_usart.pinmux_pad3 = PINMUX_UNUSED;

    while (usart_init(&usart_instance_npk, SERCOM5, &config_usart) != STATUS_OK) {
    }

    usart_enable(&usart_instance_npk);
}

/**
 * @fn			static void configure_usart_callbacks(void)
 * @brief		Code to register callbacks
 * @note
 */
static void configure_usart_callbacks(void)
{
    usart_register_callback(&usart_instance_npk, npkUsartWritecallback, USART_CALLBACK_BUFFER_TRANSMITTED);
    usart_register_callback(&usart_instance_npk, npkUsartReadcallback, USART_CALLBACK_BUFFER_RECEIVED);
    usart_enable_callback(&usart_instance_npk, USART_CALLBACK_BUFFER_TRANSMITTED);
    usart_enable_callback(&usart_instance_npk, USART_CALLBACK_BUFFER_RECEIVED);
}

/**
 * @fn			int32_t NPKSensorFreeMutex(void)
 * @brief       Frees the mutex of the given UART bus
 * @details
 * @param[in]   bus Enum that represents the bus in which we are interested to free the mutex of.
 * @return      Returns (0) if the bus is ready, (1) if it is busy.
 * @note
 */
static int32_t NPKSensorFreeMutex(void)
{
    int32_t error = ERROR_NONE;

    if (xSemaphoreGive(sensorNPKMutexHandle) != pdTRUE) {
        error = ERROR_NOT_INITIALIZED;  // We could not return the mutex! We must not have it!
    }
    return error;
}

/**
 * @fn			int32_t NPKSensorGetMutex(TickType_t waitTime)
 * @brief       Frees the mutex of the given UART bus
 * @details
 * @param[in]   waitTime Time to wait for the mutex to be freed.
 * @return      Returns (0) if the bus is ready, (1) if it is busy.
 * @note
 */
static int32_t NPKSensorGetMutex(TickType_t waitTime)
{
    int32_t error = ERROR_NONE;
    if (xSemaphoreTake(sensorNPKMutexHandle, waitTime) != pdTRUE) {
        error = ERROR_NOT_READY;
    }
    return error;
}