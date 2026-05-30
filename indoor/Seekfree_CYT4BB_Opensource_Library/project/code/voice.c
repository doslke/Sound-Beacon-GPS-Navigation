#include "zf_common_headfile.h"
#include "arm_math.h"
#include "init.h"
#include "voice.h"
#define SIGNAL_LENGTH 2048
uint16 channel_index;
#define ADC_DATA_LEN    2048
#define sample_num 2048

/*
 * adc采集完成标志位
 */
volatile uint8  AdcFinishFlag = 0;

/*
 * adc双buff缓冲区  缓冲完成的区域序号
 */
volatile uint8  AdcBuffIndex = 0;

/*
 * adc数组下标
 */
volatile uint32 adcCount = 0;

/*
 * ADC数据   adc0 1 2 3 采集麦克风信号
 *
 *     2     1
 *
 *     3     0
 */
/*int16 g_adc0Data[ADC_DATA_LEN];
int16 g_adc1Data[ADC_DATA_LEN];
int16 g_adc2Data[ADC_DATA_LEN];
int16 g_adc3Data[ADC_DATA_LEN];*/


/*
 * 互相关结果
 */
float g_acor1[30];
float g_acor2[30];
float g_acor3[30];
float g_acor4[30];

float g_Angle   = 0;

uint16 indexs=0;

uint8 get_adc_flag=0;
int16 ADC_1[sample_num];
int16 ADC_2[sample_num];
int16 ADC_3[sample_num];
int16 ADC_4[sample_num];
uint32 adds[5];
volatile int16* ADC1;
volatile int16* ADC2;
volatile int16* ADC3;
volatile int16* ADC4;
int16* A1;
int16* A2;
int16* A3;
int16* A4;
double d_1,d_2,d_3,d_4;//显示用
float dir_num,Time;
double amplitude_Sum = 0.0;
int16 norightangle(double sound_angle_1,double sound_angle_2,double sound_angle_3,double sound_angle_4)
{
    if(fabs(sound_angle_1 - sound_angle_2 )>90||fabs(sound_angle_1 - sound_angle_3 )>90
     ||fabs(sound_angle_1 - sound_angle_4 )>90||fabs(sound_angle_2 - sound_angle_3 )>90
     ||fabs(sound_angle_2 - sound_angle_4 )>90||fabs(sound_angle_3 - sound_angle_4 )>90)
        return 1;

    return 0;
}
int16 fft_sound(volatile int16 *a,volatile int16 *b)
{
    //cfloat32 fft_in_temp[sample_num],fft_out_1[sample_num],fft_out_2[sample_num];
	float inputSignal_1 [sample_num * 2];                // 定义输入信号 输入信号为复数 所以长度为 FFT_SIZE * 2
	float inputSignal_2 [sample_num * 2];               // 定义输出信号 输出信号为复数 所以长度为 FFT_SIZE * 2
 
    for(int16 i=0;i<sample_num ;i++)
    {
		inputSignal_1 [2*i]     = a[i];          // 对输入数据虚拟赋值，实际声音信号又ADC采集   将输入填入实部，虚部为0
        inputSignal_1 [2*i + 1] = 0.0;
		inputSignal_2 [2*i]     = b[i];          // 对输入数据虚拟赋值，实际声音信号又ADC采集   将输入填入实部，虚部为0
        inputSignal_2 [2*i + 1] = 0.0;
    }
	
	
	arm_cfft_instance_f32 FFT;                       // 定义FFT对象
          
    arm_cfft_init_f32(&FFT, sample_num);               // 初始化FFT对象 赋予计算长度
    arm_cfft_f32 (&FFT , inputSignal_1 , 0 , 1);        // 32位浮点FFT运算  
		
    arm_cfft_f32 (&FFT , inputSignal_2 , 0 , 1);        // 32位浮点FFT运算 
		    
	//arm_cmplx_mag_f32 (inputSignal_1 , show_mod , sample_num);                   // 将FFT结果转换为幅度谱

	/*if(amplitude_Sum == 0)
	{
		for(int16 i=0;i<sample_num;i++)
		{
	//		amplitude_Sum +=show_mod[i]/sample_num;
		}
	}*/
    //float32 re,im;
	float out [sample_num * 2];
    for(int16 i=0;i<sample_num;i++)
    {
#if 0
		show_mod [i] =  (inputSignal_1 [2*i] * inputSignal_1 [2*i] + inputSignal_1 [2*i + 1] * inputSignal_1 [2*i + 1]);	//sqrt	
#endif
		
      //  re = fft_out_1[i].real;
      //  im = fft_out_1[i].imag;

      //  fft_out_1[i].real = re * fft_out_2[i].real + im * fft_out_2[i].imag;
      //  fft_out_1[i].imag = re * fft_out_2[i].imag - im * fft_out_2[i].real;

		out[2*i]     = inputSignal_1 [2*i] * inputSignal_2 [2*i]     + inputSignal_1 [2*i + 1] * inputSignal_2 [2*i + 1];
		out[2*i + 1] = inputSignal_1 [2*i] * inputSignal_2 [2*i + 1] - inputSignal_1 [2*i + 1] * inputSignal_2 [2*i];
		
		float k = (out[2*i] * out[2*i] + out[2*i + 1] * out[2*i + 1]);
		float SCOT_1 = ((inputSignal_1 [2*i] * inputSignal_1 [2*i] + inputSignal_1 [2*i + 1] * inputSignal_1 [2*i + 1])*
							(inputSignal_2[2*i] * inputSignal_2 [2*i] + inputSignal_2 [2*i + 1] * inputSignal_2 [2*i + 1]));	
		
		float temp_k = 0.0;		//sqrt

	    arm_sqrt_f32 (k / SCOT_1 , &temp_k);
		
		out[2*i] 	  = out[2*i] * temp_k;
		out[2*i + 1] = out[2*i + 1] * temp_k ;
        //float32 k = sqrt((fft_out_1[i].real * fft_out_1[i].real + fft_out_1[i].imag * fft_out_1[i].imag);//fast_sqrt
        //fft_out_1[i].real /= pow(k,0.8);//pow(k,0.8)
        //fft_out_1[i].imag /= pow(k,0.8);//pow(k,0.8)
    }
   // Ifx_FftF32_radix2I  (fft_in_temp ,fft_out_1 ,sample_num);
		
	arm_cfft_f32 (&FFT , out , 1 , 1);        // 32位浮点IFFT运算 
	
	float out_1 [sample_num * 2];
	arm_cmplx_mag_f32 (out , out_1 , sample_num);                   // 将FFT结果转换为幅度谱

	
    int16 max_x=0;
    float m=out[0];
    for(int16 i=1;i<sample_num;i++)
    {
        float temp = out[2*i];
        if(temp>m)
        {
            m = temp;
            max_x = i;
        }
    }
    if(max_x >sample_num * 0.5)
    {
        max_x = max_x -sample_num ;
    }
    return max_x;
}


void Normal(volatile int16 *x, uint16 len)
{
    double sum = 0;
    int   i;

    for(i = 0; i < len; i++)
    {
        sum += x[i];
    }

    sum = sum / len;


    for(i = 0; i < len; i++)
    {
        x[i] -= sum;
      //  x[i] *=100;
    }
}
void getADC(){
    if(indexs < point_size) {
        flag=0;
        ADC_4[indexs] = adc_convert(ADC1_CH25_P14_5);
        ADC_3[indexs] = adc_convert(ADC1_CH00_P10_4);
        ADC_2[indexs] = adc_convert(ADC1_CH09_P12_5);
        ADC_1[indexs] = adc_convert(ADC0_CH01_P06_1);
        indexs++;
       // printf("%d,",ADC_1[indexs]);
        flag=1;
        SCB_CleanInvalidateDCache();  
        return;
      //  system_delay_us(50);
        
    }
    
    if(indexs>=point_size){
      //sound_project_fun_5()
      flag=0;
      indexs=0;
      flag=1;
      SCB_CleanInvalidateDCache();  
      return;
    }
  //flag=0;
    
}
uint8 flag=0;
volatile uint8 flagc;
void sound_project_fun_5 (void)
{

    
   // uint16 get_adc_flag =getADC((uint16*)ADC_1,(uint16*)ADC_2,(uint16*)ADC_3,(uint16*)ADC_4);
    /*if(get_adc_flag ==0){
;
      return;
    }*/
       // uint8 flags=getADC();
        //if(!flags)return;
	//timer_start (TC_TIME2_CH1);                                                  
        // 启动定时器	
    

   // printf("%d\r\n",flagc);
    if(!flagc)return;
    
    Normal(ADC1,sample_num);
    Normal(ADC2,sample_num);
    Normal(ADC3,sample_num);
    Normal(ADC4,sample_num);


    amplitude_Sum = 0;

    int16 delay12 = fft_sound(ADC4,ADC3);
    int16 delay24 = fft_sound(ADC3,ADC2);
    int16 delay31 = fft_sound(ADC1,ADC4);
    int16 delay43 = fft_sound(ADC2,ADC1);

   // printf("%d,%d,%d,%d\n",delay12,delay24,delay31,delay43);
    double sound_angle_1 = atan2(-1.0*delay12,1.0*delay24)*180/PI;
    double sound_angle_2 = atan2(-1.0*delay12,-1.0*delay31)*180/PI;
    double sound_angle_3 = atan2(1.0*delay43,1.0*delay24)*180/PI;
    double sound_angle_4 = atan2(1.0*delay43,-1.0*delay31)*180/PI;

    d_1 = sound_angle_1 ;
    d_2 = sound_angle_2 ;
    d_3 = sound_angle_3 ;
    d_4 = sound_angle_4 ;
    
	//d_1 = delay12 ;
    //d_2 = delay24 ;
    //d_3 = delay31 ;
    //d_4 = delay43 ;
	
	//Time = timer_get(TC_TIME2_CH1);                                // 获取FFT运算时长
    
    //timer_clear(TC_TIME2_CH1);                                                  // 清除定时器计数值 
    //timer_stop(TC_TIME2_CH1);    
    //Time = system_getval_ms();

    if(norightangle (sound_angle_1 ,sound_angle_2 ,sound_angle_3 ,sound_angle_4 ))
	{
		//dir_num = 150;
	    return;
	}
       
    if(delay12<=0)
        dir_num = (fabs(sound_angle_1 ) + fabs(sound_angle_2 ) + fabs(sound_angle_3 ) + fabs(sound_angle_4 ))/4;
    else
        dir_num = -(fabs(sound_angle_1 ) + fabs(sound_angle_2 ) + fabs(sound_angle_3 ) + fabs(sound_angle_4 ))/4;
	
	//dir_num = FSM_fun(dir_num);
	//if(fabs(dir_num)>100)
	//{
	//	dir_num = 150;
	//}
   // printf("%f\n",dir_num);
}
float FSM_fun(float dir_num)
{
	static float dir_array[5];
	static uint16 rear=0;
	dir_array[rear++]=dir_num;
	if(rear>=5)
		rear = 0;
	
	/*判断一致性*/
	int16 flag_add=1,flag_reduce = 1;
	for(int16 i=1;i<5;i++)
	{
		if(dir_array[i]>dir_array[i-1])
		{
			flag_reduce = 0;
		}
		else
		{
			flag_add = 0;
		}
	}
	if(flag_add||flag_reduce)
	{
		return dir_num;
	}
	
	/*计算方差*/
	float sum=0;
	for(int16 i=0;i<5;i++)
	{
		sum+=dir_array[i];
	}
	sum/=5;
	
	float var = 0;
	for(int16 i=0;i<5;i++)
	{
		var+= (dir_array[i] - sum)*(dir_array[i] - sum);
	}
	var/=5;
	
	if(var>137.5)
	{
		return 150;
	}
	else
	{
		if(fabs(dir_num)>100)
		{
			return 150;
		}
		else
			return dir_num;
	}
        
}



void debug_voice(){
    for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index ++){
        printf(
                "ADC channel %d convert data is %d.\r\n",
                channel_index + 1,
                adc_convert(channel_list[channel_index]));

    }
    system_delay_ms(1000);
}





/*!
  * @brief    ?????
  *
  * @param    acor1??  y0 y1 ???????
  * @param    acor2??  y1 y2???????
  * @param    acor3??  y0 y2???????
  * @param    acor4??  y1 y3???????
  * @param    y0   ?? ????????? y0
  * @param    y1   ?? ????????? y1
  * @param    y2   ?? ????????? y2
  * @param    y3   ?? ????????? y3
  * @param    len  ?? ????????????
  *
  * @return   ??
  *
  * @note     ??
  *
  * @see
  *
  * @date     2020/4/28
  */




/*void VoiceProcess(){
	 存放相关峰值下标 
	int16 acorIndex[4];
	Normal((int16 *)g_adc0Data, ADC_DATA_LEN);
	Normal((int16 *)g_adc1Data, ADC_DATA_LEN);
	Normal((int16 *)g_adc2Data, ADC_DATA_LEN);
	Normal((int16 *)g_adc3Data, ADC_DATA_LEN);	
	Xcorr((float *)&g_acor1, (float *)&g_acor2, (float *)&g_acor3, (float *)&g_acor4, (int16 *)&g_adc0Data, (int16 *)&g_adc1Data, (int16 *)&g_adc2Data, (int16 *)&g_adc3Data, ADC_DATA_LEN);
	SeekMaxAcor((float *)&g_acor1, (float *)&g_acor2, (float *)&g_acor3, (float *)&g_acor4, 30, acorIndex);
        ips114_clear();
        ips114_show_float(10, 10, g_acor1[acorIndex[0]], 2, 3);
         ips114_show_float(30, 30, g_acor2[acorIndex[1]], 2, 3);
          ips114_show_float(50, 50, g_acor3[acorIndex[2]], 2, 3);
           ips114_show_float(70, 70,g_acor4[acorIndex[3]], 2, 3);
           system_delay_ms(50);
	uint8 IndexMax = 0, IndexMin = 0;
	for(uint8 i = 1; i < 4; i++)
	{
		if(abs(acorIndex[i]) >= abs(acorIndex[IndexMax]))
		{
			IndexMax = i;
		}
		if(abs(acorIndex[i]) <= abs(acorIndex[IndexMin]))
		{
			IndexMin = i;
		}
	}
			if(IndexMin == 0)
		{
			if(acorIndex[1] > 0)
			{
				g_Angle = 0;
			}
			else
			{
				g_Angle = 180;
			}
		}

		else if(IndexMin == 1)
		{
			if(acorIndex[0] > 0)
			{
				g_Angle = 270;
			}
			else
			{
				g_Angle = 90;
			}
		}

		else if(IndexMin == 2)
		{
			if(acorIndex[3] > 0)
			{
				g_Angle = 45;
			}
			else
			{
				g_Angle = 225;
			}
		}

		else if(IndexMin == 3)
		{
			if(acorIndex[2] > 0)
			{
				g_Angle = 315;
			}
			else
			{
				g_Angle = 135;
			}
		}
		
                    
}*/

//LeftFront VO2 8.2
//RightHide VO1 8.1
//RightFront VO4 7.6
//RightHide VO3 7.7