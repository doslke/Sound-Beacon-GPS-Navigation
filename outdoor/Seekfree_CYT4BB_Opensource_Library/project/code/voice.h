#ifndef voice_h
#define voice_h

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"

#define point_size 2048
#define sample_num 2048
extern uint16 indexs;
extern float dir_num;
void sound_project_fun_5();
void getADC();
extern int16 ADC_1[sample_num];
extern int16 ADC_2[sample_num];
extern int16 ADC_3[sample_num];
extern int16 ADC_4[sample_num];
extern volatile int16* ADC1;
extern volatile int16* ADC2;
extern volatile int16* ADC3;
extern volatile int16* ADC4;
extern uint8 flag;
extern uint32 adds[5];
extern volatile uint8 flagc;
#endif
