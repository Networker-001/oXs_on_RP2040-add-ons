#ifndef INAV_FRSKY_H
#define INAV_FRSKY_H

#include <stdint.h>
#include <stdbool.h>

// 🟢 KORREKTUR: Wir binden die echten oXs-Core-Header ein
#include "param.h"
#include "tools.h"

//#define FRSKY_PIN1 3 
//int8_t FRSKY_PIN1 =2;
 
 
void init_frsky_sensors_core1(void);
void handleMultiProtocol_N(uint8_t b);


void read_frsky_sensors_core1(void);


void simulate_frsky_sensors_core1();
//void getSensors_ltm(void);


#endif // INAV_FRSKY_H

