#include "ssi.h"
#include "delay.h"

void SSI_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB时钟

	//GPIO输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//初始化
	
	SSI_CLK = 1;
	SSI_CSn = 1;
	
	//GPIO输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//初始化

}

uint32_t SSI_ReadPKG(void)
{
	uint8_t count = 16;
	uint32_t buf1 = 0;
	uint16_t buf2 = 0;
	SSI_CSn = 0;
	delay_us(TclkFE);
	SSI_CLK = 0;
	delay_us(Hclk);
	while(count)
	{
		buf1 <<= 1;
		buf2 <<= 1;
		SSI_CLK = 1;
		delay_us(Hclk);
		SSI_CLK = 0;
		if(SSI_DI1) buf1 ++;
		if(SSI_DI2) buf2 ++;
		delay_us(Hclk);
		count --;
	}
	SSI_CLK = 1;
	SSI_CSn = 1;
	delay_us(Tcsn);
	
	buf1 <<= 16;
	buf1 |= buf2;
	return buf1;
}

