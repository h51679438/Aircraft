#include "timer.h"
#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK STM32F407開發板
//定時器 驅動代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//創建日期:2014/5/4
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//通用定時器3中斷初始化
//arr：自動重裝值。
//psc：時鐘預分頻數
//定時器溢出時間計算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定時器工作頻率,單位:Mhz
//這裡使用的是定時器3!
void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM3時鐘
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自動重裝載值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定時器分頻
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上計數模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允許定時器3更新中斷
	TIM_Cmd(TIM2,ENABLE); //使能定時器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定時器3中斷
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //搶佔優先級1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子優先級3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//定時器3中斷服務函數
//void TIM2_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中斷
//	{
//		LED1=!LED1;//DS1翻轉
//	}
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中斷標誌位
//}
