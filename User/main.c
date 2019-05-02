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
	float pitch, roll, yaw;			//ŷ����
	short accx, accy, accz;			//���ٶ�ԭʼ����
	short gyrox, gyroy, gyroz;		//���ٶ�ԭʼ����
	float temp;						//�¶�
};


void RCC_ClockConfig(void);
void ssi_data_process(void);
uint8_t mpu_temp_pid(uint8_t hope);

const static uint8_t diameter = 5;	//ֱ�� cm
const static float Pi = 3.141592653579f;

uint32_t as1_count = 0, as2_count = 0;
uint32_t position_x = 0, position_y = 0;

struct imu_struct mpu_avr;

int main(void)
{
	RCC_ClocksTypeDef Rcc_clock;
	struct imu_struct mpu_t, mpu_b;			//mpu_addr on top and bottom
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����ϵͳ�ж����ȼ�����2
	
	#define USE_HSE
	RCC_ClockConfig();	//set sysclk to 168 MHz
	delay_init(168);
	uart_init(256000);
	LED_Init();
	IIC_Init();
	SSI_GPIO_Init();
	TIM2_IT_Init(40-1,840-1);		//��ʱ��ʱ�� APB1*2=84M����Ƶϵ��840������84M/840=100Khz�ļ���Ƶ�ʣ�����40��Ϊ400us
	TIM3_PWM_Init(500-1,84-1);		//��ʱ��ʱ�� 84M/84=1Mhz,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz.
	
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
			mpu_t.temp = MPU_Get_Temperature(mpu_t.addr);									//�õ��¶�ֵ
			MPU_Get_Accelerometer(mpu_t.addr, &mpu_t.accx, &mpu_t.accy, &mpu_t.accz);		//�õ����ٶȴ���������
			MPU_Get_Gyroscope(mpu_t.addr, &mpu_t.gyrox, &mpu_t.gyroy, &mpu_t.gyroz);		//�õ�����������
		}
		if(action_dmp_getdata(&mpu_b.addr, &mpu_b.pitch, &mpu_b.roll, &mpu_b.yaw) == 0)
		{ 
			mpu_b.temp = MPU_Get_Temperature(mpu_b.addr);									//�õ��¶�ֵ
			MPU_Get_Accelerometer(mpu_b.addr, &mpu_b.accx, &mpu_b.accy, &mpu_b.accz);		//�õ����ٶȴ���������
			MPU_Get_Gyroscope(mpu_b.addr, &mpu_b.gyrox, &mpu_b.gyroy, &mpu_b.gyroz);		//�õ�����������
		}
		
		mpu_avr.temp = (mpu_b.temp + mpu_t.temp)/2;
		mpu_avr.accx = (mpu_b.accy - mpu_t.accx)/2;
		mpu_avr.accy = (mpu_b.accx - mpu_t.accy)/2;
		mpu_avr.accz = (mpu_b.accz - mpu_t.accz)/2;
		mpu_avr.gyrox = (mpu_b.gyroy - mpu_t.gyrox)/2;
		mpu_avr.gyroy = (mpu_b.gyrox - mpu_t.gyroy)/2;
		mpu_avr.gyroz = (mpu_b.gyroz - mpu_t.gyroz)/2;
		mpu_avr.yaw = (mpu_t.yaw + mpu_b.yaw)/2;
		
//		printf("Wanl_%.1f\t%.1f\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t|%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t%d\t%d\n", 
//				Kln, mpu_avr.temp, mpu_t.accx, mpu_t.accy, mpu_t.accz, \
//				mpu_t.gyrox, mpu_t.gyroy, mpu_t.gyroz, mpu_t.pitch, mpu_t.roll, mpu_t.yaw, \
//				mpu_avr.accx, mpu_avr.accy, mpu_avr.accz, \
//				mpu_avr.gyrox, mpu_avr.gyroy, mpu_avr.gyroz, mpu_b.pitch, mpu_b.roll, mpu_avr.yaw, position_x, position_y);
		printf("%.1f\t%.1f\t%.1f\t%.1f\n", mpu_avr.temp, TKp, TKi, TKd);
		
//		USART_SendData(USART1, 0xfc);
//		USART_SendData(USART1, 0x03);
//		USART_SendData(USART1, 0x03);
//		USART_SendData(USART1, 0xfc);
//		USART_SendData(USART1, 100);
		
		delay_ms(200);
	}
}

void ssi_data_process(void)
{
	static uint16_t as1_last = 2047, as2_last = 2047;
	uint16_t as1, as2;	//AS5045 12bit abslute angle
	uint32_t astmp;
	
	astmp = SSI_ReadPKG();
	as2 = astmp;
	astmp >>= 16;
	as1 = astmp;
	as1 >>= 4;
	as2 >>= 4;
	
	if(as1_last - as1 > 3000)
	{
		as1_count ++;
		as1_last = as1;
	}
	if(as1 - as1_last > 3000)
	{
		as1_count --;
		as1_last = as1;
	}
	
	if(as2_last - as2 > 3000)
	{
		as2_count ++;
		as2_last = as2;
	}
	if(as2 - as2_last > 3000)
	{
		as2_count --;
		as2_last = as2;
	}
	
	position_x = (as1_count + as1/4096) * Pi * diameter;
	position_y = (as2_count + as2/4096) * Pi * diameter;
}

float Kln = 1.0f;
uint16_t as_data_liner(uint16_t val)
{
	return val - Kln*sin((val/2048)*2*Pi);
}


float TKp = 0.0f, TKi = 0.0f, TKd = 0.0f;
uint8_t mpu_temp_pid(uint8_t hope)
{
	static int8_t temp_last, bias_int;
	int8_t temp, bias;
	float res;
	
	temp = mpu_avr.temp;
	bias = hope - temp;
	res = TKp*bias+TKi*bias_int+TKd*(temp_last-temp);
	bias_int += bias;
	temp_last = temp;
	
	return res;
}

void RCC_ClockConfig(void)	//re-config sysclk to 168 by need
{
	RCC_DeInit();	//������ RCC�Ĵ�������Ϊȱʡֵ
	
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
    RCC_HSICmd(ENABLE);	//ʹ��HSI  
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);//�ȴ�HSIʹ�ܳɹ�
 
    //FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    //FLASH_SetLatency(FLASH_Latency_2);
   
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   
    RCC_PCLK1Config(RCC_HCLK_Div4);
    RCC_PCLK2Config(RCC_HCLK_Div2);
    RCC_PLLConfig(RCC_PLLSource_HSI, 8, 168, 2, 7);		//SYSClock = 16/8*168/2 = 168M
    RCC_PLLCmd(ENABLE);
    
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);	//�ȴ�ָ���� RCC ��־λ���óɹ� �ȴ�PLL��ʼ���ɹ�
 
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//ѡ����Ҫ��ϵͳʱ��
	
    //�ȴ�PLL�ɹ�������ϵͳʱ�ӵ�ʱ��Դ
    //  0x00��HSI ��Ϊϵͳʱ�� 
    //  0x04��HSE��Ϊϵͳʱ�� 
    //  0x08��PLL��Ϊϵͳʱ��  
    while(RCC_GetSYSCLKSource() != 0x08);//���뱻ѡ���ϵͳʱ�Ӷ�Ӧ������RCC_SYSCLKSource_PLL
	
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
