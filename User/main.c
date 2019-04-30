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

	float pitch, roll, yaw;			//ŷ����
	short accx, accy, accz;			//���ٶ�ԭʼ����
	short gyrox, gyroy, gyroz;		//���ٶ�ԭʼ����
	float Temp;						//�¶�
	
	uint16_t as1, as2;				//AS5045 12bit abslute angle
	uint32_t astmp;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����ϵͳ�ж����ȼ�����2
	RCC_ClockConfig();	//set sysclk to 168 MHz
	delay_init(168);
	uart_init(256000);
	LED_Init();
//	Heat_Res_Init();
	IIC_Init();
	SSI_GPIO_Init();
	TIM2_IT_Init(1000-1,8400-1);	//��ʱ��ʱ�� APB1*2=84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms
	TIM3_PWM_Init(500-1,840-1);		//��ʱ��ʱ�� 84M/840=100Khz,��װ��ֵ500������PWMƵ��Ϊ 100K/500=200hz.
	
	if ( MPU_Init(MPU_F_ADDR) )
	{
		LED0 = 1;
		printf("MPU6050 on TopLayer not found\n\n");
	}
	if ( MPU_Init(MPU_B_ADDR) )
	{
		LED1 = 1;
		printf("MPU6050 on BottomLayer not found\n\n");
	}
	
//	R0 = 1;
//	R1 = 1;
	
	while (1)
	{
//		while(mpu_dmp_init())
//		{
//			printf("Wanl_%d\t\tMPU6050 Error.\n", num);
//		}
//		
//		if(mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
//		{ 
			Temp = MPU_Get_Temperature(MPU_B_ADDR);					//�õ��¶�ֵ
			MPU_Get_Accelerometer(MPU_B_ADDR, &accx, &accy, &accz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(MPU_B_ADDR, &gyrox, &gyroy, &gyroz);	//�õ�����������
//		}
		
		astmp = SSI_ReadPKG();
		as2 = astmp;
		astmp >>= 16;
		as1 = astmp;
		as1 >>= 4;
		as2 >>= 4;
		
		printf("Wanl_%d\t%.1f\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t%d\t%d\n", 
				num, Temp, accx, accy, accz, gyrox, gyroy, gyroz, pitch, roll, yaw, as1, as2);
		delay_ms(50);
		
		num ++;
		if (num >= 400) num = 100;
		TIM_SetCompare1(TIM14,num);	//�޸ıȽ�ֵ
	}
}

// re-config sysclk to 168 by need
void RCC_ClockConfig(void)
{
	RCC_DeInit();
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
