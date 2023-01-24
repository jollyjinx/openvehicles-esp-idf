#include <stdint.h>
#include <string.h>

#ifndef __OVMS_CPU_START_SLEEP_H__
#define __OVMS_CPU_START_SLEEP_H__

#define BATTERY12V_WAKE_VOLTAGE_COUNT 2000
#define BATTERY12V_SLEEP_TIME 10
void singleCoreSleepBatteryIfNeeded();

extern uint32_t battery12vinfopacked1;
extern uint32_t battery12vinfopacked2;

extern uint8_t battery12vWakeVoltages[];
extern uint32_t battery12vWakeCounter;


#endif // __OVMS_CPU_START_SLEEP_H__
