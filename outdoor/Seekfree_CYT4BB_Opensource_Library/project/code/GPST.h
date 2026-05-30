#ifndef GPST_h
#define GPST_h

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"

void getpoint();
extern uint16 gnssindex;
extern volatile double lat[15];
extern volatile double lon[15];
extern uint8 nowpoint;
void followgps(uint8 pointindex,double kp);
double get_distance(uint8 index);
extern uint16 nowpointindex;
extern  uint16 everypoint[5];
extern uint16 nowpointindex;
extern uint16 points;
#endif 
