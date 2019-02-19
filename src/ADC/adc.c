#include "adc.h"
#include "delay.h"		 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK STM32F407開發板
//ADC 驅動代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//創建日期:2014/5/6
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//初始化ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA時鐘
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1時鐘

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模擬輸入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不帶上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1復位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//復位結束	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//獨立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//兩個採樣階段之間的延遲5個時鐘
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//預分頻4分頻。ADCCLK=PCLK2/4=84/4=21Mhz,ADC時鐘最好不要超過36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非掃瞄模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//關閉連續轉換
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止觸發檢測，使用軟件觸發
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右對齊	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1個轉換在規則序列中 也就是只轉換規則序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
 
	ADC_Cmd(ADC1, ENABLE);//開啟AD轉換器	

}				  
//獲得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值範圍為：ADC_Channel_0~ADC_Channel_16
//返回值:轉換結果
u16 Get_Adc(u8 ch)   
{
	  	//設置指定ADC的規則組通道，一個序列，採樣時間
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480個週期,提高採樣時間可以提高精確度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的軟件轉換啟動功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待轉換結束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1規則組的轉換結果
}
//獲取通道ch的轉換值，取times次,然後平均 
//ch:通道編號
//times:獲取次數
//返回值:通道ch的times次轉換結果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 
	 









