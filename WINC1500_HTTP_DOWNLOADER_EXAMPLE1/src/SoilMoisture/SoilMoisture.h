/*
 * SoilMoisture.h
 *
 * Created: 4/26/2023 9:32:12 PM
 *  Author: anbhati
 */ 


#ifndef SoilMoisture_H_
#define SoilMoisture_H_

#define RELAY_PIN PIN_PA21

void relay_pin_init(void);
uint16_t SoilMoisture_init(void);
uint16_t SoilMoisture_Temp(void);
uint16_t SoilMoisture_Moist(void);


#endif /* SoilMoisture_H_ */
