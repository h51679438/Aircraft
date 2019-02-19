#include "pwm.h"
#include "led.h"
#include "usart.h"
 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK STM32F407開發板
//定時器PWM 驅動代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//創建日期:2014/5/4
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//TIM14 PWM部分初始化 
//PWM輸出初始化
//arr：自動重裝值
//psc：時鐘預分頻數
void TIM14_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手動修改IO口設置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure_2;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure_1;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure_2;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure_3;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure_4;
	TIM_OCInitTypeDef  TIM_OCInitStructure_1;
	TIM_OCInitTypeDef  TIM_OCInitStructure_2;
//	TIM_OCInitTypeDef  TIM_OCInitStructure_3;
//	TIM_OCInitTypeDef  TIM_OCInitStructure_4;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);  	//TIM14時鐘使能 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTF時鐘	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3); //GPIOF9復用為定時器14
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //復用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽復用輸出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PF9
	
	GPIO_InitStructure_2.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;           //GPIOF9
	GPIO_InitStructure_2.GPIO_Mode = GPIO_Mode_AF;        //復用功能
	GPIO_InitStructure_2.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure_2.GPIO_OType = GPIO_OType_PP;      //推挽復用輸出
	GPIO_InitStructure_2.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure_2);
	  
	TIM_TimeBaseStructure_1.TIM_Prescaler=psc; 
	TIM_TimeBaseStructure_1.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure_1.TIM_Period=arr;   
	TIM_TimeBaseStructure_1.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseStructure_2.TIM_Prescaler=psc; 
	TIM_TimeBaseStructure_2.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure_2.TIM_Period=arr;   
	TIM_TimeBaseStructure_2.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure_1);
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure_2);
	
//	TIM_TimeBaseStructure_2.TIM_Prescaler=psc;  
//	TIM_TimeBaseStructure_2.TIM_CounterMode=TIM_CounterMode_Up; 
//	TIM_TimeBaseStructure_2.TIM_Period=arr;   
//	TIM_TimeBaseStructure_2.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM11,&TIM_TimeBaseStructure_2);
//	
//	TIM_TimeBaseStructure_3.TIM_Prescaler=psc;  
//	TIM_TimeBaseStructure_3.TIM_CounterMode=TIM_CounterMode_Up; 
//	TIM_TimeBaseStructure_3.TIM_Period=arr;
//	TIM_TimeBaseStructure_3.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM13,&TIM_TimeBaseStructure_3);//初始化定時器14
//	
//	TIM_TimeBaseStructure_4.TIM_Prescaler=psc;  //定時器分頻
//	TIM_TimeBaseStructure_4.TIM_CounterMode=TIM_CounterMode_Up; //向上計數模式
//	TIM_TimeBaseStructure_4.TIM_Period=arr;   //自動重裝載值
//	TIM_TimeBaseStructure_4.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseStructure_4);//初始化定時器14
	
	TIM_OCInitStructure_1.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure_1.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure_1.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OC1Init(TIM3, &TIM_OCInitStructure_1);
  
	TIM_OCInitStructure_2.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure_2.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure_2.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OC1Init(TIM4, &TIM_OCInitStructure_2);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
 
  TIM_ARRPreloadConfig(TIM3,ENABLE); 
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
//	TIM_OCInitStructure_2.TIM_OCMode = TIM_OCMode_PWM1; 
// 	TIM_OCInitStructure_2.TIM_OutputState = TIM_OutputState_Enable; 
//	TIM_OCInitStructure_2.TIM_OCPolarity = TIM_OCPolarity_Low; 
//	TIM_OC1Init(TIM11, &TIM_OCInitStructure_2);  

//	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);  
// 
//  TIM_ARRPreloadConfig(TIM11,ENABLE); 
//	
//	TIM_Cmd(TIM11, ENABLE);
//	
//	TIM_OCInitStructure_3.TIM_OCMode = TIM_OCMode_PWM1; 
// 	TIM_OCInitStructure_3.TIM_OutputState = TIM_OutputState_Enable; 
//	TIM_OCInitStructure_3.TIM_OCPolarity = TIM_OCPolarity_Low; 
//	TIM_OC1Init(TIM13, &TIM_OCInitStructure_3);  

//	TIM_OC1PreloadConfig(TIM13, TIM_OCPreload_Enable);  
// 
//  TIM_ARRPreloadConfig(TIM13,ENABLE); 
//	
//	TIM_Cmd(TIM13, ENABLE);  
//	
//	//初始化TIM14 Channel1 PWM模式	 
//	TIM_OCInitStructure_4.TIM_OCMode = TIM_OCMode_PWM1; //選擇定時器模式:TIM脈衝寬度調製模式2
// 	TIM_OCInitStructure_4.TIM_OutputState = TIM_OutputState_Enable; //比較輸出使能
//	TIM_OCInitStructure_4.TIM_OCPolarity = TIM_OCPolarity_Low; //輸出極性:TIM輸出比較極性低
//	TIM_OC1Init(TIM14, &TIM_OCInitStructure_4);  //根據T指定的參數初始化外設TIM1 4OC1

//	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的預裝載寄存器
// 
//  TIM_ARRPreloadConfig(TIM14,ENABLE);//ARPE使能 
//	
//	TIM_Cmd(TIM14, ENABLE);  //使能TIM14
 
										  
}  


