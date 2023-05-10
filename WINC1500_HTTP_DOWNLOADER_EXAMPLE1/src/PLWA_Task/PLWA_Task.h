/*
 * SoilMoisture.h
 *
 * Created: 4/26/2023 9:32:12 PM
 *  Author: anbhati
 */ 

#define PLWA_TASK_SIZE 200
#define PLWA_PRIORITY (configMAX_PRIORITIES - 1)

void vPlantWateringTask(void *pvParameters);
