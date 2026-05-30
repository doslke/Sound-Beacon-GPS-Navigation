#include "zf_common_headfile.h"

#include "voice.h"

#define CHANNEL_NUMBER (4)

#define ADC_VO1 ADC1_CH25_P14_5//1 3
#define ADC_VO2 ADC1_CH00_P10_4//2 4
#define ADC_VO3 ADC1_CH09_P12_5//3 1
#define ADC_VO4 ADC0_CH01_P06_1//4 2
#define PIT_NUM                 (PIT_CH1 ) 
#define PIT_NUM0                 (PIT_CH0 ) 
#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY4                    (P20_2)
#define KEY3                    (P20_3)
#define SWITCH1                 (P21_5)
#define SWITCH2                 (P21_6)
adc_channel_enum channel_list[CHANNEL_NUMBER] =
{
    ADC_VO1,ADC_VO2,ADC_VO3,ADC_VO4
};
uint8 voicepoints[20];
void getvoicepoint(){
      uint16 tmpnumber=1;
  while(1){
    ips114_show_int(90,70,tmpnumber,4);
      if(!gpio_get_level(KEY1))tmpnumber++;
      if(!gpio_get_level(KEY2))tmpnumber--;
      if(!gpio_get_level(KEY3))voicepoints[tmpnumber]=1;
        if(!gpio_get_level(KEY4))break;
        system_delay_ms(100);
  }


}
void init(){
   // ipcinit();
    //pwm_init(TCPWM_CH13_P00_3, 50, 285);
    //debug_info_init();
  printf("start\n");
    pwm_init(TCPWM_CH11_P01_1, 50, 740);
    pwm_init(TCPWM_CH25_P09_1, 1000, 0);
    gpio_init(P09_0, GPO, GPIO_LOW, GPI_PULL_UP);
    ips114_init();
    ips114_clear();
    //pit_ms_init(PIT_NUM0,10);
    //getpoint();
   // system_delay_ms(2000);

   // imu963ra_init();
   // system_delay_ms(2000);
   // Gyroscope_Offset_Init();
  //  system_delay_ms(500);
    //pit_ms_init(PIT_NUM,5);
   // pit_ms_init(PIT_CH2,100);
    //pit_ms_init(PIT_CH12,10);
    //system_delay_ms(5000);
    
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);
   getvoicepoint();
   // Servo_PIDinit(&servo);

    printf("Init Finished \r\n");
}

void micinit(){
    adc_init(ADC_VO1, ADC_12BIT);
    adc_init(ADC_VO2, ADC_12BIT);
    adc_init(ADC_VO3, ADC_12BIT);
    adc_init(ADC_VO4, ADC_12BIT);

}

