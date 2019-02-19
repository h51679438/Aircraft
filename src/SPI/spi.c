#include "spi.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK STM32F407開發板
//SPI 驅動代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//創建日期:2014/5/6
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//以下是SPI模塊的初始化代碼，配置成主機模式 						  
//SPI口初始化
//這裡針是對SPI1的初始化
void SPI1_Init(void)
{	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA時鐘
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1時鐘
 
  //GPIOFB3,4,5初始化設置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//PB3~5復用功能輸出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//復用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽輸出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1); //PB3復用為 SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1); //PB4復用為 SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI1); //PB5復用為 SPI1

 
	//這裡只針對SPI口初始化
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//復位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止復位SPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //設置SPI單向或者雙向的數據模式:SPI設置為雙線雙向全雙工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//設置SPI工作模式:設置為主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//設置SPI的數據大小:SPI發送接收8位幀結構
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步時鐘的空閒狀態為高電平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步時鐘的第二個跳變沿（上升或下降）數據被採樣
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信號由硬件（NSS管腳）還是軟件（使用SSI位）管理:內部NSS信號有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定義波特率預分頻的值:波特率預分頻值為256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定數據傳輸從MSB位還是LSB位開始:數據傳輸從MSB位開始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值計算的多項式
	SPI_Init(SPI1, &SPI_InitStructure);  //根據SPI_InitStruct中指定的參數初始化外設SPIx寄存器
// 
//	SPI_ITConfig(SPI1,SPI_I2S_IT_RXNE,ENABLE);
// 
//	NVIC_InitStructure.NVIC_IRQChannel=SPI1_IRQn; //定時器3中斷
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //搶佔優先級1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子優先級3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
 
	SPI_Cmd(SPI1, ENABLE); //使能SPI外設

//	NVIC_InitStructure.NVIC_IRQChannel=SPI1_IRQn; //定時器3中斷
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //搶佔優先級1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子優先級3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	SPI1_ReadWriteByte(0xff);//啟動傳輸		 
}   
//SPI1速度設置函數
//SPI速度=fAPB2/分頻係數
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2時鐘一般為84Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判斷有效性
	SPI1->CR1&=0XFFC7;//位3-5清零，用來設置波特率
	SPI1->CR1|=SPI_BaudRatePrescaler;	//設置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); //使能SPI1
} 
//SPI1 讀寫一個字節
//TxData:要寫入的字節
//返回值:讀取到的字節
u8 SPI1_ReadWriteByte(u8 TxData)
{		 			 
 
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待發送區空  
	
	SPI_I2S_SendData(SPI1, TxData); //通過外設SPIx發送一個byte  數據
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一個byte  
 
	return SPI_I2S_ReceiveData(SPI1); //返回通過SPIx最近接收的數據	
 		    
}









