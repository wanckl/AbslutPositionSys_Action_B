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
		float pitch, roll, yaw;			//ŷ����
		short accx, accy, accz;			//���ٶ�ԭʼ����
		short gyrox, gyroy, gyroz;		//���ٶ�ԭʼ����
		float temp;						//�¶�
	} mpu_t, mpu_b, mpu_avr;			//mpu_addr on top and bottom
	
	RCC_ClocksTypeDef Rcc_clock;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����ϵͳ�ж����ȼ�����2
	
	#define USE_HSE
	RCC_ClockConfig();	//set sysclk to 168 MHz
	delay_init(168);
	uart_init(256000);
	LED_Init();
	IIC_Init();
	SSI_GPIO_Init();
	TIM2_IT_Init(1000-1,8400-1);	//��ʱ��ʱ�� APB1*2=84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����1000��Ϊ100ms
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
		TIM_SetCompare3(TIM3,00);	//�޸ıȽ�ֵ
		TIM_SetCompare4(TIM3,00);
	}
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
