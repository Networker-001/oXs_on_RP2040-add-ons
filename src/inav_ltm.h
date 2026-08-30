#ifndef INAV_LTM_H
#define INAV_LTM_H

#include <stdint.h>
#include <stdbool.h>

void init_inav_ltm();
void parse_inav_ltm();

// =================================================================
// ADAPTION: EXTERN-DEKLARATION FÜR ANDERE TELEMETRIE-MODULE
// =================================================================
extern int32_t live_alt;
extern int32_t live_mvolt;
extern int32_t live_current;
extern int32_t live_speed;
extern int32_t live_sat;
extern int32_t live_rpm;
extern int32_t live_cap;
extern int32_t live_pitch;
extern int32_t live_roll;
extern int32_t live_heading;
extern uint8_t live_flight_mode;
extern uint8_t live_is_armed;

extern bool has_mvolt;
extern bool has_current;
extern bool has_speed;
extern bool has_sat;
extern bool has_rpm;
extern bool has_cap;
extern bool has_alt;


#endif