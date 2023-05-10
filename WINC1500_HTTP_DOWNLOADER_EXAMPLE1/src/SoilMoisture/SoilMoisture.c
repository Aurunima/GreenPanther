/*
 * SoilMoisture.c
 *
 * Created: 4/26/2023 9:31:59 PM
 *  Author: anbhati
 */ 

#include "WifiHandlerThread/WifiHandler.h"
#include "I2cDriver/I2cDriver.h"
#include "SerialConsole.h"
#include "SoilMoisture.h"
#include "asf.h"

//#define RELAY_PIN PIN_PA21


I2C_Data MoistureData;  ///< Global variable to use for I2C communications with the Seesaw Device

/**
 * @fn			uint16_t SoilMoisture_init(void)
 * @brief		Initialises the soil moisture sensor
 * @details		Initialises using I2C address 0x36 and registers {0x00,0x01} the soil moisture sensor
 * @note		Call from function once to initialise hardware.
 */
void relay_pin_init(void)
{
	struct port_config config_port_pin;//Define structure needed to configure a pin
	// Check if device is on the line - it should answer with its HW ID
	port_get_config_defaults(&config_port_pin); //Initialize structure with default configurations.
	//Now we change the structure to what we need.
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT; //Set pin as OUTPUT
	config_port_pin.input_pull = PORT_PIN_PULL_DOWN;
	port_pin_set_config(RELAY_PIN, &config_port_pin); //We assign the pin configuration to the PIN_PA20
	port_pin_set_output_level(RELAY_PIN, false);
	//SerialConsoleWriteString("SCREAM!");
	//delay_ms(500);
}

uint16_t SoilMoisture_init(void)
{
	uint8_t readData[2];
	uint8_t reg[2] = {0x00,0x01};
	MoistureData.address = 0x36;
	MoistureData.msgOut = &reg[0];
	MoistureData.lenOut = sizeof(reg);
	MoistureData.msgIn = readData;
	MoistureData.lenIn = sizeof(readData);
	char buf[64];
	int error = I2cReadDataWait(&MoistureData, 0xff, 0xff);
	//snprintf((char *) buf, sizeof(buf), "ID: 0x%x\r\n",readData[0]);
	//SerialConsoleWriteString(buf);
	return error;
 }

/**
 * @fn			uint16_t SoilMoisture_Temp(void)
 * @brief		Get temp from soil moisture sensor
 * @details		Get temp data using I2C address 0x36 and registers {0x00,0x04} from soil moisture sensor
 * @note		Will be called from from CLI when the command is
 *				entered in console; command is 'getMoisture'
 */
 uint16_t SoilMoisture_Temp(void)
 {
	 uint8_t readData[4];
	 uint8_t reg[2] = {0x00,0x04};
	 MoistureData.address = 0x36;
	 MoistureData.msgOut = &reg[0];
	 MoistureData.lenOut = sizeof(reg);
	 MoistureData.msgIn = readData;
	 MoistureData.lenIn = sizeof(readData);
	 int error = I2cReadDataWait(&MoistureData, 0xff, 0xff);
	 int32_t ret = ((uint32_t)readData[0] << 24) | ((uint32_t)readData[1] << 16) |  ((uint32_t)readData[2] << 8) | (uint32_t)readData[3];
	 char buf[64];
	 //snprintf((char *) buf, sizeof(buf), "Ret: %d\r\n",ret);
	 //SerialConsoleWriteString(buf);
	 //snprintf((char *) buf, sizeof(buf), "ret : %d, %d, %d, %d\r\n ", readData[0], readData[1], readData[2], readData[3]);
	 //SerialConsoleWriteString(buf);
	 float val=((1.0/(65536))*ret);
     //float val=(1.0 / (1UL << 16))*ret;
	 
	 int temp_tmp = val*100;
	 int tmp_units = temp_tmp/100;
	 int tmp_frac = temp_tmp - (tmp_units*100);

	 snprintf((char *) buf, sizeof(buf), "Temperature: %d.%d\r\n",tmp_units, tmp_frac);
	 //SerialConsoleWriteString(buf);
	 // Check if device is on the line - it should answer with its HW ID
	 WifiAddTempDataToQueue(&val);
	 
	 return error;
 }
 
 /**
 * @fn			uint16_t SoilMoisture_Moist(void)
 * @brief		Get moisture from soil moisture sensor
 * @details		Get moisture data using I2C address 0x36 and registers {0x0F,0x10} from soil moisture sensor
 * @note		Will be called from from CLI when the command is
 *				entered in console; command is 'getMoisture'
 */
 uint16_t SoilMoisture_Moist(void)
 {
	 port_pin_set_output_level(RELAY_PIN, false);
	 uint8_t readData[2];
	 uint8_t reg[2] = {0x0F,0x10};
	 MoistureData.address = 0x36;
	 MoistureData.msgOut = &reg[0];
	 MoistureData.lenOut = sizeof(reg);
	 MoistureData.msgIn = readData;
	 MoistureData.lenIn = sizeof(readData);
	 int error = I2cReadDataWait(&MoistureData, 0xff, 0xff);
	 int32_t moist = ((uint16_t)readData[0] << 8) | readData[1];
	 WifiAddMoistureDataToQueue(&moist);
	 char buf[64];
	 
	 snprintf((char *) buf, sizeof(buf), "Moisture: %d\r\n",moist);
	 //SerialConsoleWriteString(buf);
	 // Check if device is on the line - it should answer with its HW ID
	 bool rel_out = 0;
	 if (moist < 400)
	 {
		 //SerialConsoleWriteString("Dry soil!!!\r\n");
		 rel_out = true;
		 port_pin_set_output_level(RELAY_PIN, true);
		 //vTaskDelay(500);
	 }
	 else
	 {
		 //SerialConsoleWriteString("Wet soil!!!\r\n");
		 rel_out = false;
		 port_pin_set_output_level(RELAY_PIN, 0);
		 //vTaskDelay(500);
	 }
	 //WifiAddSprinklerDataToQueue(&rel_out);
	 return error;
 }
