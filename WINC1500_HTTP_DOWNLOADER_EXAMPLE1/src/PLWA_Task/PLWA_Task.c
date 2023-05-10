/*
 * PLWA_Task.c
 *
 * Created: 4/26/2023 9:31:59 PM
 *  Author: anbhati
 */ 

#include "asf.h"
#include "SerialConsole.h"
#include "PLWA_Task/PLWA_Task.h"
#include "SoilMoisture\SoilMoisture.h"

void vPlantWateringTask(void *pvParameters)
{
	char buf[64];
	uint16_t status = SoilMoisture_init();
	
	while(1)
	{
		status = SoilMoisture_Temp();
		status = SoilMoisture_Moist();
		vTaskDelay(1000);
	}
}
