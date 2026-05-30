#ifndef __PID_h_
#define __PID_h_
#include "zf_common_headfile.h"
typedef struct
{
 double target;	//目标值
 double now; //当前值
 double p,i,d;
 double error[3];	//上一次的kp误差、用于计算kd误差
 int dout;
 int out;   //输出

}PID;

extern PID servo;


extern uint8 pd_flag;

void Servo_PIDinit(PID *pid);
void ServoOut(PID *pid);//PID位置式控制器输出
void ServoPID(uint16 thisnumber);//舵机输出



#endif
