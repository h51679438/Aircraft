#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "spi.h"
#include "key.h" 
#include "24l01.h"
#include "adc.h"
#include "pwm.h"
#include "timer.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

//ALIENTEK 探索者STM32F407開發板 實驗33
//NRF24L02無線通信實驗-庫函數版本
//技術支持：www.openedv.com
//淘寶店舖：http://eboard.taobao.com  
//廣州市星翼電子科技有限公司  
//作者：正點原子 @ALIENTEK
 
//要寫入到W25Q16的字符串數組
//const u8 TEXT_Buffer[]={"Explorer STM32F4 SPI TEST"};
//#define SIZE sizeof(TEXT_Buffer)	 
	
//u8 tmp_buf[33]={0};
//u16 led0pwmval_1=20000*37/40;
//u16 led0pwmval_2=20000*37/40;
//u16 led0pwmval_3=20000*37/40;
//u16 led0pwmval_4=20000*37/40;
//u8 servomotor_reference_1,servomotor_reference_2,servomotor_reference_3,servomotor_reference_4;
//void SPI1_IRQHandler(void)
//{
//	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_IT_RXNE)==SET)
//	{
//		if(NRF24L01_RxPacket(tmp_buf)==0)
//			{
//				servomotor_reference_1=tmp_buf[0];
//				servomotor_reference_2=tmp_buf[1];
//				servomotor_reference_3=tmp_buf[2];
//				servomotor_reference_4=tmp_buf[3];
//				TIM3->CCR1=led0pwmval_1;
//				TIM3->CCR2=led0pwmval_2;
//				TIM3->CCR3=led0pwmval_3;
//				TIM3->CCR4=led0pwmval_4; 
//			}
//	}
//	SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
//}
//void RX_First(u8*servomotor_reference_1,u8*servomotor_reference_2,u8*servomotor_reference_3,u8*servomotor_reference_4)
//{
//	u8 tmp_buf[33]={0};
//	while(1)
//	{
//		if(NRF24L01_RxPacket(tmp_buf)==0)
//		{
//			*servomotor_reference_1=tmp_buf[0];
//			*servomotor_reference_2=tmp_buf[1];
//			*servomotor_reference_3=tmp_buf[2];
//			*servomotor_reference_4=tmp_buf[3];
//			break;
//		}
//	}
//}
void svm_control(u16*svm,u16*init_svm)
{
	if(*svm >= (*init_svm)+200)
		*svm = (*init_svm)+200;
	if(*svm<=(*init_svm)-200)
		*svm =(*init_svm)-200;
}
void brushless_control(u16*blm)
{
	if(*blm >= 19050)
		*blm = 19050;
	if(*blm<=16000)
		*blm =16000;
	
}
void direction_control(u8*temp)
{
	if(*temp<=0X00)
		*temp = 1;
	else if(*temp>=0x0F)
		*temp = 2;
	else
		*temp =0;
}
void svm_front(u16*svm,u16*init_svm,u8 angle)
{
	TIM_SetCompare1(TIM3,(*svm+0)+angle);
	delay_us(20);
	TIM_SetCompare2(TIM3,(*svm+1)-angle);
	delay_us(20);
	TIM_SetCompare3(TIM3,(*svm+2)+angle);
	delay_us(20);
	TIM_SetCompare4(TIM3,(*svm+3)-angle);
	delay_us(20);
	svm_control((svm),(init_svm));
	svm_control((svm+1),(init_svm+1));
	svm_control((svm+2),(init_svm+2));
	svm_control((svm+3),(init_svm+3));
}
void svm_behind(u16*svm,u16*init_svm,u8 angle)
{
	TIM_SetCompare1(TIM3,*(svm)-angle);
	delay_us(20);
	TIM_SetCompare2(TIM3,*(svm+1)+angle);
	delay_us(20);
	TIM_SetCompare3(TIM3,*(svm+2)-angle);
	delay_us(20);
	TIM_SetCompare4(TIM3,*(svm+3)+angle);
	delay_us(20);
	svm_control((svm),(init_svm));
	svm_control((svm+1),(init_svm+1));
	svm_control((svm+2),(init_svm+2));
	svm_control((svm+3),(init_svm+3));
}
void svm_left(u16*svm,u16*init_svm,u8 angle)
{
	TIM_SetCompare1(TIM3,*(svm)+angle);
	delay_us(20);
	TIM_SetCompare2(TIM3,*(svm+1)-angle);
	delay_us(20);
	TIM_SetCompare3(TIM3,*(svm+2)-angle);
	delay_us(20);
	TIM_SetCompare4(TIM3,*(svm+3)+angle);
	delay_us(20);
	svm_control((svm),(init_svm));
	svm_control((svm+1),(init_svm+1));
	svm_control((svm+2),(init_svm+2));
	svm_control((svm+3),(init_svm+3));
}
void svm_right(u16*svm,u16*init_svm,u8 angle)
{
	TIM_SetCompare1(TIM3,*(svm)-angle);
	delay_us(20);
	TIM_SetCompare2(TIM3,*(svm+1)+angle);
	delay_us(20);
	TIM_SetCompare3(TIM3,*(svm+2)+angle);
	delay_us(20);
	TIM_SetCompare4(TIM3,*(svm+3)-angle);
	delay_us(20);
	svm_control((svm),(init_svm));
	svm_control((svm+1),(init_svm+1));
	svm_control((svm+2),(init_svm+2));
	svm_control((svm+3),(init_svm+3));
}

void self_control_svm(u16*init_svm,u16*svm,float*euler)
{
	u16 op_svm[2];
	u16 op_svm_1[4];
	u16 op[4];
	
	
	float op_1[4];
	
	if(*(euler)<0)
		op_1[0] = -*(euler);
	else
		op_1[0] = *(euler);
	op_1[1] = op_1[0]*200;
	op[0] = (u16)(op_1[1]);
	
	op_svm[0] = op[0]>>5 ;
	
	
	if(*(euler+1)<0)
		op_1[2] = -*(euler+1); 
	else
		op_1[2] = *(euler+1);
	op_1[3] = op_1[2]*200;
	op[2] = (u16)(op_1[3]);
	
	op_svm[1] = ((u16)(36000)-op[2])>>5;
	if(*(euler)<0)
	{
		op_svm_1[0] = (*(init_svm)+op_svm[0]);
		op_svm_1[1] = (*(init_svm+1)-op_svm[0]);
		op_svm_1[2] = (*(init_svm+2)-op_svm[0]);
		op_svm_1[3] = (*(init_svm+3)+op_svm[0]);
	}
	else
	{
		op_svm_1[0] = (*(init_svm)-op_svm[0]);
		op_svm_1[1] = (*(init_svm+1)+op_svm[0]);
		op_svm_1[2] = (*(init_svm+2)+op_svm[0]);
		op_svm_1[3] = (*(init_svm+3)-op_svm[0]);
	}
	if(*(euler+1)<0)
	{
		*(svm) = (op_svm_1[0]-op_svm[1]);
		*(svm+1) = (op_svm_1[1]+op_svm[1]);
		*(svm+2) = (op_svm_1[2]-op_svm[1]);
		*(svm+3) = (op_svm_1[3]+op_svm[1]);
	}
	else
	{
		*(svm) = (op_svm_1[0]+op_svm[1]);
		*(svm+1) = (op_svm_1[1]-op_svm[1]);
		*(svm+2) = (op_svm_1[2]+op_svm[1]);
		*(svm+3) = (op_svm_1[3]-op_svm[1]);
	}
	svm_control((svm),(init_svm));
	svm_control((svm+1),(init_svm+1));
	svm_control((svm+2),(init_svm+2));
	svm_control((svm+3),(init_svm+3));
//	TIM_SetCompare1(TIM3,(*svm));
//	delay_us(20);
//	TIM_SetCompare2(TIM3,(*svm+1));
//	delay_us(20);
//	TIM_SetCompare3(TIM3,(*svm+2));
//	delay_us(20);
//	TIM_SetCompare4(TIM3,(*svm+3));
//	delay_us(20);
	
}


 double KalmanFilter_0(const double ResrcData_0,double ProcessNiose_Q_0,double MeasureNoise_R_0)          //because procedure , I change the static double to double
{

    double R = MeasureNoise_R_0;
    double Q = ProcessNiose_Q_0;

    static double x_last_0;
    double x_mid = x_last_0;
    double x_now;

    static double p_last_0;
    double p_mid ;
    double p_now;

    double kg;

    x_mid=x_last_0;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last_0+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??

    /*
     *  ????????????
     */
    kg=p_mid/(p_mid+R);                 //kg?kalman filter?R ???
    x_now=x_mid+kg*(ResrcData_0-x_mid);   //???????
    p_now=(1-kg)*p_mid;                 //??????covariance
    p_last_0 = p_now;                     //??covariance ?
    x_last_0 = x_now;                     //???????
    return x_now;
}
double KalmanFilter_1(const double ResrcData_1,double ProcessNiose_Q_1,double MeasureNoise_R_1)          //because procedure , I change the static double to double
{

    double R = MeasureNoise_R_1;
    double Q = ProcessNiose_Q_1;

    static double x_last_1;
    double x_mid = x_last_1;
    double x_now;

    static double p_last_1;
    double p_mid ;
    double p_now;

    double kg;

    x_mid=x_last_1;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last_1+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??

    /*
     *  ????????????
     */
    kg=p_mid/(p_mid+R);                 //kg?kalman filter?R ???
    x_now=x_mid+kg*(ResrcData_1-x_mid);   //???????
    p_now=(1-kg)*p_mid;                 //??????covariance
    p_last_1 = p_now;                     //??covariance ?
    x_last_1 = x_now;                     //???????
    return x_now;
}
int main(void)
{
	float pitch,roll,yaw; 		//歐拉角
	short aacx,aacy,aacz;		//加速度傳感器原始數據
	short gyrox,gyroy,gyroz;	//陀螺儀原始數據
	short temp;					//溫度
	
	u8 value,angle;
	u8 t=0;
	u16 a=20000;
	u8 mode=1;
	u16 adcx;			 
	u8 tmp_buf[33]={1,2,3,4,5,6,7,8,9,10,
									11,12,13,14,15,16,17,18,19,20,
									21,22,23,24,25,26,27,28,29,30,
									31,32,33};	
	u16 blm_above = 19050;
	u16 blm_below = 19050;
	u8 control = 0;
									
	float euler[3];
	u16 init_svm[4];
	u16 svm[4]={18500,18500,18500,18500};
									
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//設置系統中斷優先級分組2
	delay_init(168);  //初始化延時函數
//	uart_init(115200);	//初始化串口波特率為115200
	LED_Init();					//初始化LED 
// 	LCD_Init();					//LCD初始化  
// 	KEY_Init();					//按鍵初始化
 	NRF24L01_Init();    		//初始化NRF24L01 
//	Adc_Init();         //初始化ADC
	TIM14_PWM_Init(a-1,84-1);
									
	while(NRF24L01_Check())
	{
		
	}	 
	if(mode)//RX模式
	{	  
		MPU_Init();
		NRF24L01_RX_Mode();
		while(mpu_dmp_init())
		{
		}
		while(control!=0)
		{
			NRF24L01_RxPacket(tmp_buf);
			control =tmp_buf[4];
		}
		while(t<1)
		{
			if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
			{
				temp=MPU_Get_Temperature();	//得到溫度值
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度傳感器數據
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺儀數據
				t++;
			}
		}
		euler[0]=0;
		euler[1]=180;
		euler[2]=yaw;
		t=0;
		TIM_SetCompare1(TIM3,svm[0]);
		delay_us(20);
		TIM_SetCompare2(TIM3,svm[1]);
		delay_us(20);
		TIM_SetCompare3(TIM3,svm[2]);
		delay_us(20);
		TIM_SetCompare4(TIM3,svm[3]);
		delay_us(20);
		while(tmp_buf[3]!=1)
		{
			NRF24L01_RxPacket(tmp_buf);
			if(tmp_buf[2]==1)
			{
				if(tmp_buf[1]==1)
					svm[0]++;
				else if(tmp_buf[1]==2)
					svm[0]--;
				else if(tmp_buf[0]==1)
					svm[1]++;
				else if(tmp_buf[0]==2)
					svm[1]--;
			}
			else if(tmp_buf[2]==2)
			{
				if(tmp_buf[1]==1)
					svm[2]++;
				else if(tmp_buf[1]==2)
					svm[2]--;
				else if(tmp_buf[0]==1)
					svm[3]++;
				else if(tmp_buf[0]==2)
					svm[3]--;
			}
			TIM_SetCompare1(TIM3,svm[0]);
			delay_ms(2);
			TIM_SetCompare2(TIM3,svm[1]);
			delay_ms(2);
			TIM_SetCompare3(TIM3,svm[2]);
			delay_ms(2);
			TIM_SetCompare4(TIM3,svm[3]);
			delay_ms(2);
		}
		init_svm[0]=svm[0];
		init_svm[1]=svm[1];
		init_svm[2]=svm[2];
		init_svm[3]=svm[3];
		TIM_SetCompare1(TIM4,17600);
		TIM_SetCompare2(TIM4,17600);
		delay_ms(2000);
		TIM_SetCompare1(TIM4,19050);
		TIM_SetCompare2(TIM4,19050);
		while(1)
		{
			if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
			{
				temp=MPU_Get_Temperature();	//得到溫度值
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度傳感器數據
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺儀數據
			}
			if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,則顯示出來.
			{
				angle=20;
				NRF24L01_RxPacket(tmp_buf);
				if(tmp_buf[0]==1)
					svm_front(svm,init_svm,angle);
				if(tmp_buf[0]==2)
					svm_behind(svm,init_svm,angle);
				if(tmp_buf[1]==1)
					svm_left(svm,init_svm,angle);
				if(tmp_buf[1]==2)
					svm_right(svm,init_svm,angle);
				
				if(tmp_buf[3]==1)
				{
					TIM_SetCompare1(TIM4,blm_above=blm_above-50);
					TIM_SetCompare2(TIM4,blm_below=blm_below-50);
					brushless_control(&blm_above);
					brushless_control(&blm_below);
				}
				else if(tmp_buf[3]==2)
				{
					TIM_SetCompare1(TIM4,blm_above=blm_above+20);
					TIM_SetCompare2(TIM4,blm_below=blm_below+20);
					brushless_control(&blm_above);
					brushless_control(&blm_below);
				}
				if(tmp_buf[2]==1)
				{
					TIM_SetCompare1(TIM4,blm_above=blm_above-50);
					brushless_control(&blm_above);
				}
				else if(tmp_buf[2]==2)
				{
					TIM_SetCompare1(TIM4,blm_above=blm_above+20);
					brushless_control(&blm_above);
				}
				
				delay_us(10);
			}				    
			else 
			{
				pitch=KalmanFilter_0((double)pitch,1,1.5);
				roll=KalmanFilter_1((double)roll,1,1.5);
				
				euler[0] = pitch;
				euler[1] = roll;
				self_control_svm(init_svm,svm,euler);
				TIM_SetCompare1(TIM3,svm[0]);
				delay_us(20);
				TIM_SetCompare2(TIM3,svm[1]);
				delay_us(20);
				TIM_SetCompare3(TIM3,svm[2]);
				delay_us(20);
				TIM_SetCompare4(TIM3,svm[3]);
				delay_us(20);
			}
		};	
	}else//TX模式
	{
		NRF24L01_TX_Mode();
		while(1)
		{
			tmp_buf[4]=5;
			if(KEY_Scan(1)!=0)
			{
				tmp_buf[4]=0;
			}
			adcx=Get_Adc_Average(ADC_Channel_4,5);//獲取通道5的轉換值，20次取平均
			value=(u8)adcx>>4;
			direction_control(&value);
			tmp_buf[0]=value;
			adcx=Get_Adc_Average(ADC_Channel_5,5);
			value=(u8)adcx>>4;
			direction_control(&value);
			tmp_buf[1]=value;
			adcx=Get_Adc_Average(ADC_Channel_6,5);
			value=(u8)adcx>>4;
			direction_control(&value);
			tmp_buf[2]=value;
			adcx=Get_Adc_Average(ADC_Channel_7,5);
			value=(u8)adcx>>4;
			direction_control(&value);
			tmp_buf[3]=value;
			if(tmp_buf[0]!=0x00||tmp_buf[1]!=0x00||tmp_buf[2]!=0x00||tmp_buf[3]!=0x00)
			{
				NRF24L01_TxPacket(tmp_buf);
			}
						    
		};
	}     
		
}

