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
void svm_control(u16*svm)
{
	if(*svm >= 18800)
		*svm = 18800;
	if(*svm<=18200)
		*svm =18200;
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
	if(*temp<=0X01)
		*temp = 1;
	else if(*temp>=0x0E)
		*temp = 2;
	else
		*temp =0;
}

int main(void)
{ 
	u8 key,temp;
	u16 a=20000;
	u8 mode=0;
	u16 t=0,adcx;			 
	u8 tmp_buf[33]={1,2,3,4,5,6,7,8,9,10,
									11,12,13,14,15,16,17,18,19,20,
									21,22,23,24,25,26,27,28,29,30,
									31,32,33};	
	u16 svm_1=18500;
	u16 svm_2=18500;
	u16 svm_3=18500;
	u16 svm_4=18500;
	u16 blm_above = 19050;
	u16 blm_below = 19050;
	u8 svm_rf_1=0,svm_rf_2=0,blm_rf_1=0,blm_rf_2=0;
	u8 control = 0;
									
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//設置系統中斷優先級分組2
	delay_init(168);  //初始化延時函數
//	uart_init(115200);	//初始化串口波特率為115200
	LED_Init();					//初始化LED 
// 	LCD_Init();					//LCD初始化  
// 	KEY_Init();					//按鍵初始化
 	NRF24L01_Init();    		//初始化NRF24L01 
	Adc_Init();         //初始化ADC
//	TIM14_PWM_Init(a-1,84-1);
									
//	TIM_SetCompare1(TIM4,17600);			
//	TIM_SetCompare2(TIM4,17600);
//	delay_ms(2000);
//	TIM_SetCompare1(TIM4,19450);			
//	TIM_SetCompare2(TIM4,19450);
	
	while(NRF24L01_Check())
	{
		
	}	 
	if(mode==1)
		NRF24L01_RX_Mode();
	if(mode)//RX模式
	{	  
//		RX_First(&servomotor_reference_1,&servomotor_reference_2,&servomotor_reference_3,&servomotor_reference_4);
//		while(NRF24L01_RxPacket(tmp_buf)!=0);
//		svm_rf_1=tmp_buf[0];
//		svm_rf_2=tmp_buf[1];
//		blm_rf_1=tmp_buf[2];
//		blm_rf_2=tmp_buf[3];
		while(control!=0)
		{
			NRF24L01_RxPacket(tmp_buf);
			control =tmp_buf[4];
		}
		TIM_SetCompare1(TIM3,18500);
		TIM_SetCompare2(TIM3,18500);
		TIM_SetCompare3(TIM3,18500);
		TIM_SetCompare4(TIM3,18500);
		while(tmp_buf[3]!=1)
			NRF24L01_RxPacket(tmp_buf);
		TIM_SetCompare1(TIM4,17600);
		TIM_SetCompare2(TIM4,17600);
		delay_ms(2000);
		TIM_SetCompare1(TIM4,19050);
		TIM_SetCompare2(TIM4,19050);
		while(1)
		{	  		    		    				 
			if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,則顯示出來.
			{
				delay_us(10);
//				svm_1=svm_1+((svm_rf_1-tmp_buf[0])*10);
//				svm_control(&svm_1);
//				svm_2=svm_2+((svm_rf_2-tmp_buf[1])*10);
//				svm_control(&svm_2);
//				blm_above=blm_above+((blm_rf_1-tmp_buf[2])*10);
//				brushless_control(&blm_above);
//				blm_below=blm_below+((blm_rf_2-tmp_buf[3])*10);
//				brushless_control(&blm_below);
				
				
				if(tmp_buf[0]==1)
				{
					TIM_SetCompare1(TIM3,svm_1=svm_1+20);
					TIM_SetCompare2(TIM3,svm_2=svm_2-20);
					TIM_SetCompare3(TIM3,svm_3=svm_3+20);
					TIM_SetCompare4(TIM3,svm_4=svm_4-20);
					svm_control(&svm_1);
					svm_control(&svm_2);
					svm_control(&svm_3);
					svm_control(&svm_4);
				}
				if(tmp_buf[0]==2)
				{
					TIM_SetCompare1(TIM3,svm_1=svm_1-20);
					TIM_SetCompare2(TIM3,svm_2=svm_2+20);
					TIM_SetCompare3(TIM3,svm_3=svm_3-20);
					TIM_SetCompare4(TIM3,svm_4=svm_4+20);
					svm_control(&svm_1);
					svm_control(&svm_2);
					svm_control(&svm_3);
					svm_control(&svm_4);
				}
				if(tmp_buf[1]==1)
				{
					TIM_SetCompare1(TIM3,svm_1=svm_1-20);
					TIM_SetCompare2(TIM3,svm_2=svm_2+20);
					TIM_SetCompare3(TIM3,svm_3=svm_3+20);
					TIM_SetCompare4(TIM3,svm_4=svm_4-20);
					svm_control(&svm_1);
					svm_control(&svm_2);
					svm_control(&svm_3);
					svm_control(&svm_4);
				}
				if(tmp_buf[1]==2)
				{
					TIM_SetCompare1(TIM3,svm_1=svm_1+20);
					TIM_SetCompare2(TIM3,svm_2=svm_2-20);
					TIM_SetCompare3(TIM3,svm_3=svm_3-20);
					TIM_SetCompare4(TIM3,svm_4=svm_4+20);
					svm_control(&svm_1);
					svm_control(&svm_2);
					svm_control(&svm_3);
					svm_control(&svm_4);
				}
//				if(tmp_buf[0]==0||tmp_buf[1]==0)
//				{
//					svm_1 = 18500;
//					svm_2 = 18500;
//					svm_3 = 18500;
//					svm_4 = 18500;
//				}
				
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
				if(tmp_buf[4]==1)
				{
					TIM_SetCompare1(TIM4,blm_above=blm_above-50);
					brushless_control(&blm_above);
				}
				else if(tmp_buf[4]==2)
				{
					TIM_SetCompare1(TIM4,blm_above=blm_above+20);
					brushless_control(&blm_above);
				}
				
				delay_us(50);
//					LED0=!LED0;
			}
			else 
				delay_us(100);	   
//			if(t==10000)//大約1s鍾改變一次狀態
//			{
//				t=0;
//				LED0=!LED0;
//			} 				    
		};	
	}else//TX模式
	{							    
		NRF24L01_TX_Mode();
		while(1)
		{	 
			tmp_buf[4]=4;
			if(KEY_Scan(1)!=0)
			{
				tmp_buf[4]=0;
			}
			adcx=Get_Adc_Average(ADC_Channel_4,20);//獲取通道5的轉換值，20次取平均
			temp=(u8)adcx>>4;
			direction_control(&temp);
			tmp_buf[0]=temp;
			adcx=Get_Adc_Average(ADC_Channel_5,20);
			temp=(u8)adcx>>4;
			direction_control(&temp);
			tmp_buf[1]=temp;
			adcx=Get_Adc_Average(ADC_Channel_6,20);
			temp=(u8)adcx>>4;
			direction_control(&temp);
			tmp_buf[2]=temp;
			adcx=Get_Adc_Average(ADC_Channel_7,20);
			temp=(u8)adcx>>4;
			direction_control(&temp);
			tmp_buf[3]=temp;
			if(NRF24L01_TxPacket(tmp_buf)==TX_OK)
			{
				LED0=!LED0;
			}
			
			delay_ms(1);				    
		};
	}     
		
}

