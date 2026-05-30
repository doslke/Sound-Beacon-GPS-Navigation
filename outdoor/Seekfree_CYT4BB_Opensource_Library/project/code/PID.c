#include "PID.h"
#include "zf_common_headfile.h"
#include "Gyroscope_solve.h"
#include "GPST.h"
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)

/*
28000 170 620 0.6 32s
32000 155 610 0.7 28s
*/

uint8 pd_flag;



#define mid   740                  //2ms
#define r_max 520
#define l_max 1020                  //左右限幅不要打死 容易舵机卡住


PID servo;
void Servo_PIDinit(PID *pid)
{


    pid->target=0;
    pid->p=3.5;
    pid->i=0;
    pid->d=40;
    pid->out=0;
    pid->now=0;
    pid->error[0]=0;
    pid->error[1]=0;
    pid->error[2]=0;


}


float abs_fa(float a){
  return a>=0?a:-a;
}







void ServoPID(uint16 thisnumber)//舵机控制
{
        
             servo.now=gnss.direction;
	      servo.target=get_two_points_azimuth(gnss.latitude,gnss.longitude,lat[thisnumber],lon[thisnumber]);    
        ServoOut(&servo);
        pwm_set_duty(TCPWM_CH11_P01_1, servo.out);  //更新占空比
}





void ServoOut(PID *pid)//位置式 PID控制
{
    
    if(pid->now>0&&pid->target<=pid->now-180&&pid->target>-180)
    {
        pid->error[0]=360-pid->now+pid->target;
    }else
    if(pid->now<0&&pid->target>=180+pid->now&&pid->target<180)
    {
        pid->error[0]=-360+pid->target-pid->now;
    }
    else
    {
        pid->error[0]=pid->target - pid->now;
    }  //当前误差
		//pid->error[0]=-pid->error[0];
    pid->out=mid-(int)(pid->p*pid->error[0]+pid->d*(pid->error[0]-pid->error[1]));   //舵机输出
    pid->error[1] = pid->error[0];                                    //上一次误差
    if(pid->out>=l_max)     //限制最大角度
    {
        pid->out=l_max;
    }
    if(pid->out<=r_max)     //限制最大角度
    {
        pid->out=r_max;
    }
}

