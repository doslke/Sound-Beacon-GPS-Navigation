#ifndef init_h
#define init_h

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"
#define CHANNEL_NUMBER (4)
extern adc_channel_enum channel_list[CHANNEL_NUMBER];
extern uint8 voicepoints[20];
void init();
void micinit();
#endif

