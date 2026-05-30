

#include "zf_common_headfile.h"


#include "voice.h"
#include "init.h"


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

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	
    debug_info_init();                 
    
    
    //ctltime=1;
   system_delay_ms(500);                                                  
    
    flash_init();                                                               
    
    flash_read_page_to_buffer(M7_1_FLASH_SECTION_INDEX, M7_1_FLASH_PAGE_INDEX, 5);                
     ADC1 = (volatile int16*)flash_union_buffer[0].uint32_type;          
     ADC2 = (volatile int16*)flash_union_buffer[1].uint32_type;           
     ADC3 = (volatile int16*)flash_union_buffer[2].uint32_type;            
     ADC4 = (volatile int16*)flash_union_buffer[3].uint32_type;            
     flagc=(volatile uint8)*(uint8*)flash_union_buffer[4].uint32_type;
    pit_ms_init(PIT_CH2,100);
    fifo_init(&uart_data_fifo,FIFO_DATA_8BIT,uart_get_data,64);
    
    uart_init(UART_1,115200,UART1_TX_P04_1,UART1_RX_P04_0);
    uart_rx_interrupt(UART_1,1);
    uart_init(UART_2,115200,UART2_TX_P10_1,UART2_RX_P10_0);
    uart_rx_interrupt(UART_1,1);
    init();
    kp=3.5;
    system_delay_ms(1000);
   float angle=0;
   uint8 i=1;
   uint8 flag=1;
    while(true)
    {
   
    ips114_show_int(30,70,speed,4);
      SCB_CleanInvalidateDCache(); 
       ips114_show_int(10,70,speed,4);
      sound_project_fun_5();
        ips114_show_float(90,70,dir_num,4,3);
        float outangle=dir_num;
        if(dir_num<=-100||dir_num>=100){
           outangle=180-abs_f(dir_num);
           if(dir_num>100)
             outangle=outangle*-1;
           gpio_high(P09_0);
        }
        else{
          gpio_low(P09_0);
        }
         out=740-kp*(outangle);
          if(out>1020)out=1020;
          if(out<510)out=510;
           pwm_set_duty(TCPWM_CH11_P01_1, out);
           if(abs_f(dir_num-100)<5)
           system_delay_ms(200);
           else if(abs_f(dir_num)<5||abs_f(dir_num)>175&&thisnumber>=1){
                
                pwm_set_duty(TCPWM_CH25_P09_1,1500);
                
           }else if(thisnumber<1||voicepoints[thisnumber]==0)pwm_set_duty(TCPWM_CH25_P09_1,0);
             else pwm_set_duty(TCPWM_CH25_P09_1,speed);
      
      
       ips114_clear();
     
    
    if(thisnumber<1||voicepoints[thisnumber]==0)pwm_set_duty(TCPWM_CH25_P09_1,0);
     if( !gpio_get_level(KEY1))       
        {
            speed+=10;
        }
      if(!gpio_get_level(KEY2))         
        {
            speed-=10;
           
        }

     
    }}

// **************************** ???????? ****************************
