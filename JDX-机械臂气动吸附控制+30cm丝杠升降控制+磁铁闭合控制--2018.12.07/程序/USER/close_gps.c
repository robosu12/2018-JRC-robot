#include "stm32f10x.h"
#include "close_GPS.h"

uchar  common_start[]={0xB5,0x62,6,1,8,0,0xF0};
uchar  msk_gga[]={0,0,0,0,0,0,1,0,0x24};	//屏蔽GPS_GGA消息
//uchar code opn_gga[]={0,0,1,0,0,0,1,1,0x29};	//打开GPS_GGA消息
uchar  msk_gll[]={1,0,0,0,0,0,1,1,0x2B};
//uchar code opn_gll[]={1,0,1,0,0,0,1,2,0x30};
uchar  msk_gsa[]={2,0,0,0,0,0,1,2,0x32};
//uchar code opn_gsa[]={2,0,1,0,0,0,1,3,0x37};
uchar  msk_gsv[]={3,0,0,0,0,0,1,3,0x39};
//uchar code opn_gsv[]={3,0,1,0,0,0,1,4,0x3E};
uchar  msk_rmc[]={4,0,0,0,0,0,1,4,0x40};
//uchar code opn_rmc[]={4,0,1,0,0,0,1,5,0x45};
uchar  msk_vtg[]={5,0,0,0,0,0,1,5,0x47};
//uchar code opn_vtg[]={5,0,1,0,0,0,1,6,0x4C};
void mask_cominfo(void)
{
	uchar i;
	for(i=0;i<7;i++)
	{
		USART_SendData(USART1,(u16)common_start[i]);
		while(USART_GetITStatus(USART1,USART_IT_TXE)==0);
		USART_ClearFlag(USART1,USART_IT_TXE );
	} 
}

void mask_selinfo(uchar *str_sel)
{
	uchar i;
	mask_cominfo();
	for(i=0;i<9;i++)
	{
		USART_SendData(USART1,(u16)str_sel[i]);
		while(USART_GetITStatus(USART1,USART_IT_TXE)==0);
		USART_ClearFlag(USART1,USART_IT_TXE );
	}
}

void mask_GPS(void)
{
	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
	mask_selinfo(msk_gga);
	mask_selinfo(msk_gll);
	mask_selinfo(msk_gsa);
	mask_selinfo(msk_gsv);
	//mask_selinfo(msk_rmc);
	mask_selinfo(msk_vtg);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
}


