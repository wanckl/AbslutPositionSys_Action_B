#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "iic.h"
#include "ssi.h"
#include "led.h"
#include "timer.h"
#include "pwm.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

void RCC_ClockConfig(void);

int main(void)
{
	uint16_t num = 100;
	uint16_t as1, as2;	//AS5045 12bit abslute angle
	uint32_t astmp;
	
	struct imu_struct
	{
		uint8_t addr;
		float pitch, roll, yaw;			//欧拉角
		short accx, accy, accz;			//加速度原始数据
		short gyrox, gyroy, gyroz;		//角速度原始数据
		float temp;						//温度
	} mpu_t, mpu_b, mpu_avr;			//mpu_addr on top and bottom
	
	RCC_ClocksTypeDef Rcc_clock;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置系统中断优先级分组2
	
	#define USE_HSE
	RCC_ClockConfig();	//set sysclk to 168 MHz
	delay_init(168);
	uart_init(256000);
	LED_Init();
	IIC_Init();
	SSI_GPIO_Init();
	TIM2_IT_Init(1000-1,8400-1);	//定时器时钟 APB1*2=84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数1000次为100ms
	TIM3_PWM_Init(500-1,84-1);		//定时器时钟 84M/84=1Mhz,重装载值500，所以PWM频率为 1M/500=2Khz.
	
	mpu_t.addr = MPU_Top;
	mpu_b.addr = MPU_Bottom;
	
	delay_ms(500);	//wait for imu-sensor power stable
	if( action_dmp_init(mpu_t.addr) )
	{
		LED0 = 1;
		printf("MPU6050 on TopLayer not found\n\n");
	}
	if( action_dmp_init(mpu_b.addr) )
	{
		LED1 = 1;
		printf("MPU6050 on BottomLayer not found\n\n");
	}
	
	while(1)
	{
		RCC_GetClocksFreq(&Rcc_clock);
		if(action_dmp_getdata(&mpu_t.addr, &mpu_t.pitch, &mpu_t.roll, &mpu_t.yaw) == 0)
		{ 
			mpu_t.temp = MPU_Get_Temperature(mpu_t.addr);									//得到温度值
			MPU_Get_Accelerometer(mpu_t.addr, &mpu_t.accx, &mpu_t.accy, &mpu_t.accz);		//得到加速度传感器数据
			MPU_Get_Gyroscope(mpu_t.addr, &mpu_t.gyrox, &mpu_t.gyroy, &mpu_t.gyroz);		//得到陀螺仪数据
		}
		if(action_dmp_getdata(&mpu_b.addr, &mpu_b.pitch, &mpu_b.roll, &mpu_b.yaw) == 0)
		{ 
			mpu_b.temp = MPU_Get_Temperature(mpu_b.addr);									//得到温度值
			MPU_Get_Accelerometer(mpu_b.addr, &mpu_b.accx, &mpu_b.accy, &mpu_b.accz);		//得到加速度传感器数据
			MPU_Get_Gyroscope(mpu_b.addr, &mpu_b.gyrox, &mpu_b.gyroy, &mpu_b.gyroz);		//得到陀螺仪数据
		}
		
		mpu_avr.temp = (mpu_b.temp + mpu_t.temp)/2;
		mpu_avr.accx = (mpu_b.accy - mpu_t.accx)/2;
		mpu_avr.accy = (mpu_b.accx - mpu_t.accy)/2;
		mpu_avr.accz = (mpu_b.accz - mpu_t.accz)/2;
		mpu_avr.gyrox = (mpu_b.gyroy - mpu_t.gyrox)/2;
		mpu_avr.gyroy = (mpu_b.gyrox - mpu_t.gyroy)/2;
		mpu_avr.gyroz = (mpu_b.gyroz - mpu_t.gyroz)/2;
		
		astmp = SSI_ReadPKG();
		as2 = astmp;
		astmp >>= 16;
		as1 = astmp;
		as1 >>= 4;
		as2 >>= 4;
		
		printf("Wanl_%d\t%.1f\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t|%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t%d\t%d\n", 
				num, mpu_avr.temp, mpu_t.accx, mpu_t.accy, mpu_t.accz, \
				mpu_t.gyrox, mpu_t.gyroy, mpu_t.gyroz, mpu_t.pitch, mpu_t.roll, mpu_t.yaw, \
				mpu_avr.accx, mpu_avr.accy, mpu_avr.accz, \
				mpu_avr.gyrox, mpu_avr.gyroy, mpu_avr.gyroz, mpu_b.pitch, mpu_b.roll, mpu_b.yaw, as1, as2);
		
		delay_ms(100);
		num ++;
		if (num >= 300) num = 100;
		TIM_SetCompare3(TIM3,00);	//修改比较值
		TIM_SetCompare4(TIM3,00);
	}
}


void RCC_ClockConfig(void)	//re-config sysclk to 168 by need
{
	RCC_DeInit();	//将外设 RCC寄存器重设为缺省值
	
#if defined USE_HSE
	RCC_HSEConfig( RCC_HSE_ON );
	if(SUCCESS == RCC_WaitForHSEStartUp())    
	{
		
		RCC_HCLKConfig  (RCC_SYSCLK_Div1);
		RCC_PCLK1Config (RCC_HCLK_Div4);
		RCC_PCLK2Config (RCC_HCLK_Div2);
		RCC_PLLConfig   (RCC_PLLSource_HSE, 6, 168, 2, 7);	//SYSClock = 12/6*168/2 = 168M

		RCC_PLLCmd(ENABLE);

		while(RCC_GetFlagStatus ( RCC_FLAG_PLLRDY) == RESET )
		{
		}

		RCC_SYSCLKConfig  ( RCC_SYSCLKSource_PLLCLK );

		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}

#elif defined USE_HSI
    RCC_HSICmd(ENABLE);	//使能HSI  
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);//等待HSI使能成功
 
    //FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    //FLASH_SetLatency(FLASH_Latency_2);
   
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   
    RCC_PCLK1Config(RCC_HCLK_Div4);
    RCC_PCLK2Config(RCC_HCLK_Div2);
    RCC_PLLConfig(RCC_PLLSource_HSI, 8, 168, 2, 7);		//SYSClock = 16/8*168/2 = 168M
    RCC_PLLCmd(ENABLE);
    
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);	//等待指定的 RCC 标志位设置成功 等待PLL初始化成功
 
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//选择想要的系统时钟
	
    //等待PLL成功用作于系统时钟的时钟源
    //  0x00：HSI 作为系统时钟 
    //  0x04：HSE作为系统时钟 
    //  0x08：PLL作为系统时钟  
    while(RCC_GetSYSCLKSource() != 0x08);//需与被选择的系统时钟对应起来，RCC_SYSCLKSource_PLL
	
#endif
}


#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
		ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
  * @}
  */


/**************** (C) COPYRIGHT STMicroelectronics *****END OF FILE*****************/
