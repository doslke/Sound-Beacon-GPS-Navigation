

#include "zf_common_headfile.h"

#include "PID.h"
#include "voice.h"
#include "init.h"
#include "Gyroscope_solve.h"
#include "GPST.h"
#define M0_FLASH_SECTION_INDEX       (0)                                   
#define M0_FLASH_PAGE_INDEX          (0)                                      

#define M7_1_FLASH_SECTION_INDEX     (0)                                        
#define M7_1_FLASH_PAGE_INDEX        (1)                                        
#define KEY1                    (P20_0)
#define KEY2                    (P20_1)

#define KEY3                    (P20_2)
#define KEY4                    (P20_3)
//int16* ADC1;
uint8 addindex=0;

uint32 send_data_test = 0;
uint16 speed=0;
uint8 getindex;
uint16 out=0;
double yawsl=0;
uint8 uart_get_data[64];                                                      
uint8 fifo_get_data[64];                                                      
fifo_struct uart_data_fifo;
extern int16 thisnumber;
uint8  get_data = 0;     

void uart_rx_interrupt_handler(void){
  if(uart_query_byte(UART_1,&get_data)){
      fifo_write_buffer(&uart_data_fifo,&get_data,1);
  
  }


}

int16 getnumber(void){
    uint32 fifo_data_count=fifo_used(&uart_data_fifo);
    if(fifo_data_count!=0){
        fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
    }
    else return-1;
    
    for(int16 i=0;i<60;i++){
      if(fifo_get_data[i]==0x66){
        if(fifo_get_data[i+3]==0x88){
          return fifo_get_data[i+1]; 
        
        }
      }
    }


}

int tmpnnumber=0;
extern double kp;
uint8 changeindex=0;
extern uint8 ctltime;
extern uint8 ctlflag;

float abs_f(float a){
  return a>=0?a:-a;
}
#define mid 740
#define lmax 1020
#define rmax 520
uint16 speedv=850;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	
    debug_info_init();                 
    uart_init(UART_2,115200,UART2_TX_P10_1,UART2_RX_P10_0);
    uart_rx_interrupt(UART_2,1);
    init();

   thisnumber=1;
    while(true)
    {
     ips114_show_int(10,90,speed,4);
    if((get_two_points_distance(lat[thisnumber],lon[thisnumber],gnss.latitude,gnss.longitude)<=2)||get_two_points_azimuth(gnss.latitude,gnss.longitude,lat[thisnumber],lon[thisnumber])>90){
    thisnumber++;
    }
    if(thisnumber>=points){
      thisnumber=1;
      tmpnnumber=1;
    }
    if( !gpio_get_level(KEY1))       
        {
            speed+=10;
        }
      if(!gpio_get_level(KEY2))         
        {
            speed-=10;
           
        }
    pwm_set_duty(TCPWM_CH25_P09_1,speed);
    if(tmpnnumber==1&&thisnumber>1)pwm_set_duty(TCPWM_CH25_P09_1,0);
    }
}
// **************************** ???????? ****************************
