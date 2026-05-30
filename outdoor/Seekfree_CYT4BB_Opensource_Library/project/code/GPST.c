#include "zf_common_headfile.h"
#include "Gyroscope_solve.h"
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)
#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define SWITCH1                 (P21_5)
#define SWITCH2                 (P21_6)
volatile double lat[30],lon[30];
uint16 gnssindex=0;
uint8 nowpoint=0;
uint8 H,L;
uint8 pointnumber=1;
uint16 everypoint[5];
uint16 points;
void getpoint(){
    uint8 i=0;
    uint16 tmppoint=0;
    gnss_init(TAU1201);//gps初始化
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);        // 初始化 SWITCH1 输入 默认高电平 上拉输入
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);        // 初始化 SWITCH2 输入 默认高电平 上拉输入
    while(1){
     //   gnss_data_parse();
        //ips114_clear();
        ips114_show_float(10,30,gnss.latitude,4,6);
        ips114_show_float(10,50,gnss.longitude,4,6);
        ips114_show_int(10,70,gnss.satellite_used,4);
        if( !gpio_get_level(KEY1))       
        {
            tmppoint+=1;
            system_delay_ms(50);
        }
      if(!gpio_get_level(KEY2))         
        {
            tmppoint-=1;
            system_delay_ms(50);
        } 
        ips114_clear();
        ips114_show_int(10,10,tmppoint,4);
        //system_delay_ms(100);
        if(!gpio_get_level(KEY3)){//按钮3按下开始采集
          system_delay_ms(250);
            double tmpla=0;
            double tmplo=0;
            for(i=0;i<50;i++){
                gnss_data_parse();
                tmpla+=gnss.latitude;
                tmplo+=gnss.longitude;
            }
 
            lat[tmppoint]=(double)tmpla/50.00;
            lon[tmppoint]=(double)tmplo/50.00;
            points=tmppoint;
        }
         if(!gpio_get_level(KEY4)){
            
      break;
     }    
    }
}
float abs_fa1(float a){
  return a>=0?a:-a;
}

void followgps(uint8 pointindex,double kp){
  
    float yawsl=get_two_points_azimuth(gnss.latitude,gnss.longitude,lat[pointindex],lon[pointindex])-gnss.direction;
   // ips114_show_float(10,90,get_two_points_azimuth(gnss.latitude,gnss.longitude,lat[pointindex],lon[pointindex]),4,1);
  //  ips114_show_float(90,90,eulerAngle.yaw,4,1);
     int16 out=0;
     if(yawsl>180){yawsl=-360+yawsl;}
     if(yawsl<-180){yawsl=360+yawsl;}
     float outangle=yawsl; 
     
     if(yawsl<=-90||yawsl>=90){
           outangle=180-abs_fa1(yawsl);
           if(yawsl>90)
             outangle=outangle*-1;
           gpio_high(P09_0);
           //电机反转
        }
        else{
          //电机正转
          gpio_low(P09_0);
        }
   //  ips114_show_float(90,70,outangle,4,3);
     out=830-(outangle)*kp;
     if(out>1020)out=1030;
     if(out<510)out=520;
     pwm_set_duty(TCPWM_CH11_P01_1,out);
     return;
    
}