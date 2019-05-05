#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
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

struct imu_struct
{
	uint8_t addr;
	float pitch, roll, yaw;			//欧拉角
	short accx, accy, accz;			//加速度原始数据
	short gyrox, gyroy, gyroz;		//角速度原始数据
	float temp;						//温度
};


void RCC_ClockConfig(void);
void ssi_data_process(void);
uint16_t as_data_liner(uint16_t val);
uint8_t mpu_temp_pid(uint8_t hope);

const static uint8_t diameter = 50;	//直径 mm
const static float Pi = 3.141592653579f;

int32_t as1_count = 0, as2_count = 0;
int32_t position_x = 0, position_y = 0;
int32_t as1_zero, as2_zero;

struct imu_struct mpu_avr;

int main(void)
{
	uint8_t mput_fail = 0, mpub_fail = 0;
	uint8_t send_led = 2;	//send frequncy div2 to tonggle blink led
	
	RCC_ClocksTypeDef Rcc_clock;
	struct imu_struct mpu_t, mpu_b;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置系统中断优先级分组2
	
	#define USE_HSE
	RCC_ClockConfig();	//set sysclk to 168 MHz
	delay_init(168);
	uart_init(57600);
	LED_Init();
	IIC_Init();
	SSI_GPIO_Init();
	TIM3_PWM_Init(500-1,84-1);		//定时器时钟 84M/84=1Mhz,重装载值500，所以PWM频率为 1M/500=2Khz.
	RCC_GetClocksFreq(&Rcc_clock);
	
	mpu_t.addr = MPU_Top;
	mpu_b.addr = MPU_Bottom;
	delay_ms(400);	//wait for imu-sensor power stable
	if( action_dmp_init(mpu_t.addr) )
	{
		mput_fail = 1;
		printf("MPU6050 on TopLayer not found\n\n");
	}
	if( action_dmp_init(mpu_b.addr) )
	{
		mpub_fail = 1;
		printf("MPU6050 on BottomLayer not found\n\n");
	}
	
	if(mput_fail || mpub_fail)	//Retry
	{
		MPU_Write_Byte(mpu_t.addr, MPU_PWR_MGMT1_REG, 0X80);	//复位MPU6050
		MPU_Write_Byte(mpu_b.addr, MPU_PWR_MGMT1_REG, 0X80);
		delay_ms(300);
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
	}
	
	TIM2_IT_Init(40-1,840-1);		//定时器时钟 APB1*2=84M，分频系数840，所以84M/840=100Khz的计数频率，计数40次为400us
	while(1)
	{
		if(imu_get_flag)	//imu 10ms & 100Hz get data flag
		{
			imu_get_flag = 0;
			
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
			mpu_avr.yaw = (mpu_t.yaw + mpu_b.yaw)/2;
		}
		
//		if(display_flag)
//		{
//			display_flag = 0;
//			
//			printf("Wanl_%.1f\t%.1f\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t|%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t%d\t%d\n", 
//					Kln, mpu_avr.temp, mpu_t.accx, mpu_t.accy, mpu_t.accz, \
//					mpu_t.gyrox, mpu_t.gyroy, mpu_t.gyroz, mpu_t.pitch, mpu_t.roll, mpu_t.yaw, \
//					mpu_avr.accx, mpu_avr.accy, mpu_avr.accz, \
//					mpu_avr.gyrox, mpu_avr.gyroy, mpu_avr.gyroz, mpu_b.pitch, mpu_b.roll, mpu_avr.yaw, position_x, position_y);
//			printf("%.1f\t%.1f\t%.1f\t%.1f\n", mpu_avr.temp, TKp, TKi, TKd);
//			
//		}
		
		send_led --;
		if(!send_led)
		{
			send_led = 2;
			LED1 = ~LED1;
		}
		delay_ms(8);
	}
}

void ssi_data_process(void)
{
	static uint16_t as1_last = 2047, as2_last = 2047, as1_start, as2_start;
	uint16_t as1, as2;	//AS5045 12bit abslute angle
	uint32_t astmp;
	const float sample = 4095.0f;
	static uint8_t if_first = 1;
	
	astmp = SSI_ReadPKG();
	as2 = astmp;
	astmp >>= 16;
	as1 = astmp;
	as1 >>= 4;
	as2 >>= 4;
	
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕 
//	USART_SendData(USART1, 0xfc);
//	while((USART1->SR&0X40)==0);
//	USART_SendData(USART1, 0x03);
//	while((USART1->SR&0X40)==0);
//	USART_SendData(USART1, 0x03);
//	while((USART1->SR&0X40)==0);
//	USART_SendData(USART1, 0xfc);

//	printf("as1=%d\tas1_count=%d\tposition_x=%d\n", as1, as1_count, position_x);

//	while((USART1->SR&0X40)==0);
//	USART_SendData(USART1, position_x);
//	while((USART1->SR&0X40)==0);
//	USART_SendData(USART1, position_x>>8);
//	as1 = as_data_liner(as1);
//	while((USART1->SR&0X40)==0);
//	USART_SendData(USART1, position_x>>16);
//	while((USART1->SR&0X40)==0);
//	USART_SendData(USART1, position_x>>24);

	if(as1_last - as1 > 3000)
	{
		as1_count ++;
	}
	if(as1 - as1_last > 3000)
	{
		as1_count --;
	}
	
	if(as2_last - as2 > 3000)
	{
		as2_count ++;
	}
	if(as2 - as2_last > 3000)
	{
		as2_last = as2;
	}
	
	if(if_first)
	{
		if_first = 0;
		as1_start = as1;
		as2_start = as2;
	}
	
	as1_last = as1;
	as2_last = as2;
	as1_zero = as1 - as1_start;
	as2_zero = as2 - as2_start;
	
	position_x = (as1_count + as1_zero/sample) * Pi * diameter;
	position_y = (as2_count + as2_zero/sample) * Pi * diameter;
}

int8_t Kln = 120;
uint16_t as_data_liner(uint16_t val)
{
	const float cycle = 2048.0f;
	return (val + (Kln * sin((val/cycle)*2*Pi)));
}


float TKp = 30.0f, TKi = 0.5f, TKd = 0.0f;
uint8_t mpu_temp_pid(uint8_t hope)
{
	static int8_t temp_last, bias_int;
	int8_t temp, bias;
	float res;
	
	temp = mpu_avr.temp;
//	printf("%.1f\n", mpu_avr.temp);
	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕 
	USART_SendData(USART1, 0xfc);
	while((USART1->SR&0X40)==0);
	USART_SendData(USART1, 0x03);
	while((USART1->SR&0X40)==0);
	USART_SendData(USART1, 0x03);
	while((USART1->SR&0X40)==0);
	USART_SendData(USART1, 0xfc);
	while((USART1->SR&0X40)==0);
	USART_SendData(USART1, temp);
	
	bias = hope - temp;
	res = TKp*bias+TKi*bias_int+TKd*(temp_last-temp);
	bias_int += bias;
	temp_last = temp;
	
	if(temp == hope) bias_int = 0;
	if(res > 200)	res = 200;
	if(res < 0)		res = 0;
	return res;
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
