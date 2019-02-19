#ifndef __24L01_H
#define __24L01_H	 		  
#include "sys.h"   
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK STM32F407開發板
//NRF24L01驅動代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//修改日期:2014/5/9
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//NRF24L01寄存器操作命令
#define NRF_READ_REG    0x00  //讀配置寄存器,低5位為寄存器地址
#define NRF_WRITE_REG   0x20  //寫配置寄存器,低5位為寄存器地址
#define RD_RX_PLOAD     0x61  //讀RX有效數據,1~32字節
#define WR_TX_PLOAD     0xA0  //寫TX有效數據,1~32字節
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.發射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包數據,CE為高,數據包被不斷發送.
#define NOP             0xFF  //空操作,可以用來讀狀態寄存器	 
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0發射模式;bit1:電選擇;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中斷MAX_RT(達到最大重發次數中斷)使能;bit5:中斷TX_DS使能;bit6:中斷RX_DR使能
#define EN_AA           0x01  //使能自動應答功能  bit0~5,對應通道0~5
#define EN_RXADDR       0x02  //接收地址允許,bit0~5,對應通道0~5
#define SETUP_AW        0x03  //設置地址寬度(所有數據通道):bit1,0:00,3字節;01,4字節;02,5字節;
#define SETUP_RETR      0x04  //建立自動重發;bit3:0,自動重發計數器;bit7:4,自動重發延時 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道頻率;
#define RF_SETUP        0x06  //RF寄存器;bit3:傳輸速率(0:1Mbps,1:2Mbps);bit2:1,發射功率;bit0:低噪聲放大器增益
#define STATUS          0x07  //狀態寄存器;bit0:TX FIFO滿標誌;bit3:1,接收數據通道號(最大:6);bit4,達到最多次重發
                              //bit5:數據發送完成中斷;bit6:接收數據中斷;
#define MAX_TX  		0x10  //達到最大發送次數中斷
#define TX_OK   		0x20  //TX發送完成中斷
#define RX_OK   		0x40  //接收到數據中斷

#define OBSERVE_TX      0x08  //發送檢測寄存器,bit7:4,數據包丟失計數器;bit3:0,重發計數器
#define CD              0x09  //載波檢測寄存器,bit0,載波檢測;
#define RX_ADDR_P0      0x0A  //數據通道0接收地址,最大長度5個字節,低字節在前
#define RX_ADDR_P1      0x0B  //數據通道1接收地址,最大長度5個字節,低字節在前
#define RX_ADDR_P2      0x0C  //數據通道2接收地址,最低字節可設置,高字節,必須同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //數據通道3接收地址,最低字節可設置,高字節,必須同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //數據通道4接收地址,最低字節可設置,高字節,必須同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //數據通道5接收地址,最低字節可設置,高字節,必須同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //發送地址(低字節在前),ShockBurstTM模式下,RX_ADDR_P0與此地址相等
#define RX_PW_P0        0x11  //接收數據通道0有效數據寬度(1~32字節),設置為0則非法
#define RX_PW_P1        0x12  //接收數據通道1有效數據寬度(1~32字節),設置為0則非法
#define RX_PW_P2        0x13  //接收數據通道2有效數據寬度(1~32字節),設置為0則非法
#define RX_PW_P3        0x14  //接收數據通道3有效數據寬度(1~32字節),設置為0則非法
#define RX_PW_P4        0x15  //接收數據通道4有效數據寬度(1~32字節),設置為0則非法
#define RX_PW_P5        0x16  //接收數據通道5有效數據寬度(1~32字節),設置為0則非法
#define NRF_FIFO_STATUS 0x17  //FIFO狀態寄存器;bit0,RX FIFO寄存器空標誌;bit1,RX FIFO滿標誌;bit2,3,保留
                              //bit4,TX FIFO空標誌;bit5,TX FIFO滿標誌;bit6,1,循環發送上一數據包.0,不循環;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01操作線
#define NRF24L01_CE   PGout(6) 	//24L01片選信號
#define NRF24L01_CSN  PGout(7) 	//SPI片選信號	   
#define NRF24L01_IRQ  PGin(8)  	//IRQ主機數據輸入
//24L01發送接收數據寬度定義
#define TX_ADR_WIDTH    5   	//5字節的地址寬度
#define RX_ADR_WIDTH    5   	//5字節的地址寬度
#define TX_PLOAD_WIDTH  32  	//32字節的用戶數據寬度
#define RX_PLOAD_WIDTH  32  	//32字節的用戶數據寬度
									   	   

void NRF24L01_Init(void);//初始化
void NRF24L01_RX_Mode(void);//配置為接收模式
void NRF24L01_TX_Mode(void);//配置為發送模式
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 u8s);//寫數據區
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 u8s);//讀數據區		  
u8 NRF24L01_Read_Reg(u8 reg);			//讀寄存器
u8 NRF24L01_Write_Reg(u8 reg, u8 value);//寫寄存器
u8 NRF24L01_Check(void);//檢查24L01是否存在
u8 NRF24L01_TxPacket(u8 *txbuf);//發送一個包的數據
u8 NRF24L01_RxPacket(u8 *rxbuf);//接收一個包的數據
#endif











