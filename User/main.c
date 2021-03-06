#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "iic.h"
#include "mpu9250.h"
#include "ssi.h"
#include "led.h"
#include "timer.h"
#include "pwm.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ahrs.h"


void RCC_ClockConfig(void);
void ssi_data_process(void);
uint16_t as_data_liner(uint16_t val);
uint8_t mpu_temp_pid(uint8_t hope);
void usart1_send_char(u8 c);
void usart1_ANO_report(u8 fun,u8*data,u8 len);
void average_fliter(imu_struct *fliter_in, imu_struct *fliter_out);
void upload_data(int32_t px, int32_t py, int32_t vx, int32_t vy, short pitch, short roll, short yaw, short temp);

const static uint8_t diameter = 50;	//直径 mm
const static float Pi = 3.141592653579f;
float global_temp;
imu_struct mpu_out;

int32_t position_x = 0, position_y = 0;
double gravity;

int main(void)
{
	uint8_t send_led = 4;		//send frequncy div2 to tonggle blink led
	
//	float angle_x, angle_y, angle_z;
//	short angle_send_x = 0, angle_send_y = 0, angle_send_z = 0;
	int32_t position_x_last = 0, position_y_last = 0, velocity_x = 0, velocity_y = 0;
//	float velocity_imu_x = 0, velocity_imu_y = 0, position_imu_x = 0, position_imu_y = 0;

	RCC_ClocksTypeDef Rcc_clock;
	imu_struct mpu_orig;
	short mag_x, mag_y, mag_z;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置系统中断优先级分组2
	
	#define USE_HSE
	RCC_ClockConfig();	//set sysclk to 168 MHz
	delay_init(168);
	uart_init(115200);
	LED_Init();
	IIC_Init();
	SSI_GPIO_Init();
	TIM3_PWM_Init(500-1,84-1);		//定时器时钟 84M/84=1Mhz,重装载值500，所以PWM频率为 1M/500=2Khz.
	RCC_GetClocksFreq(&Rcc_clock);
	
	mpu_orig.addr = MPU_ADDR;
	delay_ms(400);	//wait for imu-sensor power stable
	MPU_Init(mpu_orig.addr);
//	if( mpu_dmp_init() )
//	{
//		printf("MPU6050 INIT FAIL\n");
//	}
	
//	delay_ms(2000);
//	IIC2_Init();
//	MPU2_Init(0x68);
	
	TIM2_IT_Init(40-1,840-1);		//定时器时钟 APB1*2=84M，分频系数840，所以84M/840=100Khz的计数频率，计数40次为400us
	while(1)
	{
		if(velocity_flag)	//calculate speed use position data, unit: mm/s, update freq: 2Hz
		{
			velocity_flag = 0;
			velocity_x = (position_x - position_x_last) << 1;
			velocity_y = (position_y - position_y_last) << 1;
			position_x_last = position_x;
			position_y_last = position_y;
			
			upload_data(position_x, position_y, velocity_x, velocity_y, mpu_orig.pitch, mpu_orig.roll, mpu_orig.yaw, mpu_orig.temp*10);
		}
		
		if(imu_get_flag)	//imu 10ms & 100Hz get data flag
		{
			imu_get_flag = 0;
			
//			if(action_dmp_getdata(&mpu_orig.pitch, &mpu_orig.roll, &mpu_orig.yaw, &mpu_orig))
//			{
//				printf("mpu_dmp_read_faild!\n");
//			}
			
			mpu_orig.temp = MPU_Get_Temperature(mpu_orig.addr)-20;
			MPU_Get_Gyroscope(mpu_orig.addr, &mpu_orig.gyrox, &mpu_orig.gyroy, &mpu_orig.gyroz);
			MPU_Get_Accelerometer(mpu_orig.addr, &mpu_orig.accx, &mpu_orig.accy, &mpu_orig.accz);
			MPU9250_GetMag(&mag_x, &mag_y, &mag_z);
			
//			mpu_out.temp = mpu_b.temp;
//			global_temp = mpu_b.temp;
//			average_fliter(&mpu_b, &mpu_out);
//			
//			get_euler_angle(mpu_out.gyrox, mpu_out.gyroy, mpu_out.gyroz, mpu_out.accx, mpu_out.accy, mpu_out.accz, \
																		&mpu_out.pitch, &mpu_out.roll, &mpu_out.yaw);
			
//			velocity_imu_x += (((mpu_out.accx - gravity*asin(mpu_out.roll*A2R))/32768)*2*gravity)*0.01f;
//			velocity_imu_y += (((mpu_out.accy - gravity*asin(mpu_out.pitch*A2R))/32768)*2*gravity)*0.01f;
//			position_imu_x += velocity_imu_x*0.01f;
//			position_imu_y += velocity_imu_y*0.01f;

//			angle_x += mpu_t.gyrox*0.01;
//			angle_y += mpu_t.gyroy*0.01;
//			angle_z += -mpu_out.gyroz*0.01*GYRO_PRE;
//			angle_send_z = angle_z + 0.5;	//四舍五入
//			
//			usart1_send_char(0xfc);
//			usart1_send_char(0x03);
//			usart1_send_char(0x03);
//			usart1_send_char(0xfc);
//			usart1_send_char(mpu_t.gyroz);
//			usart1_send_char(mpu_t.gyroz>>8);
//			usart1_send_char(mpu_out.gyrox);
//			usart1_send_char(mpu_out.gyrox>>8);
//			usart1_send_char(angle_send_z);
//			usart1_send_char(angle_send_z>>8);
			
			send_led --;
			if(!send_led)
			{
				send_led = 4;
				LED1 = ~LED1;
//				
//				printf("Wanl_%.1f\t%.2f\t%.2f\t%.2f\t| %d\t%d\t%d\t%d\t%d\t%d\t| %d\t%d\t%d\n", \
//											mpu_out.temp, mpu_out.pitch, mpu_out.roll, mpu_out.yaw, \
//												mpu_out.accx, mpu_out.accy, mpu_out.accz, \
//												mpu_out.gyrox, mpu_out.gyroy, mpu_out.gyroz, \
//												mag_x, mag_y, mag_z);
//				
////				printf("%.2f\t%.2f\t| %.2f\t%.2f\t%.2f\t%.2f\n", mpu_out.roll, mpu_out.pitch, \
////								velocity_imu_x, velocity_imu_y, position_imu_x, position_imu_y);
//				
//				printf("%.1f\t%d\t%d\t%d\n", mpu_b.temp, mag_x, mag_y, mag_z);
			}
		}
		delay_ms(2);
	}
}

void ssi_data_process(void)
{
	static uint8_t if_first = 1;
	static uint16_t as1_last, as2_last;
	uint16_t as1_now, as2_now, as1_inc, as2_inc;	//AS5045 12bit abslute angle
	uint32_t astmp;
	const float sample = 4096.0f;
	
	astmp = SSI_ReadPKG();
	as2_now = astmp;
	astmp >>= 16;
	as1_now = astmp;
	as1_now >>= 4;
	as2_now >>= 4;

	if(if_first)
	{
		if_first = 0;
		as1_last = as1_now;
		as2_last = as2_now;
	}

	if(as1_last - as1_now > 3000)	//检测是否越过零点跳变
	{
		as1_inc = sample - as1_last + as1_now;
	}
	if(as1_now - as1_last > 3000)
	{
		as1_inc = -(sample - as1_now + as1_last);
	}
	
	if(as2_last - as2_now > 3000)
	{
		as2_inc = sample - as2_last + as2_now;
	}
	if(as2_now - as2_last > 3000)
	{
		as2_inc = -(sample - as2_now + as2_last);
	}

	as1_inc = as1_now - as1_last;		//求出增量值
	as2_inc = as2_now - as2_last;
	
	as1_last = as1_now;		//刷新上一次记录
	as2_last = as2_now;
	
	position_x += ((as1_inc*cos(mpu_out.yaw)-as2_inc*sin(mpu_out.yaw))*diameter)/sample;		//投影并积分增量值, 单位: mm
	position_y += ((as1_inc*sin(mpu_out.yaw)+as2_inc*cos(mpu_out.yaw))*diameter)/sample;
}

int8_t Kln = 120;
uint16_t as_data_liner(uint16_t val)
{
	const float cycle = 2048.0f;
	return (val + (Kln * sin((val/cycle)*2*Pi)));
}


float TKp = 30.0f, TKi = 0.6f, TKd = 0.0f;
uint8_t mpu_temp_pid(uint8_t hope)
{
	static int8_t temp_last, bias_int;
	int8_t temp, bias;
	float res;
	
	temp = global_temp;
//	printf("%.1f\n", mpu_avr.temp);
	
//	usart1_send_char(0xfc);
//	usart1_send_char(0x03);
//	usart1_send_char(0x03);
//	usart1_send_char(0xfc);
//	usart1_send_char(temp);
	
	bias = hope - temp;
	res = TKp*bias+TKi*bias_int+TKd*(temp_last-temp);
	bias_int += bias;
	temp_last = temp;
	
	if(temp == hope) bias_int = 0;
	if(res > 200)	res = 200;
	if(res < 0)		res = 0;
	return res;
}

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_ANO_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1
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

void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET)
	{
	}
    USART_SendData(USART1, c);   

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

void average_fliter(imu_struct *fliter_in, imu_struct *fliter_out)
{
	static const uint8_t fliter_lenth = 12;
	static int64_t int_accx, int_accy, int_accz, int_gyrox, int_gyroy, int_gyroz;
	static short accel[3][fliter_lenth], gyro[3][fliter_lenth];
	static uint8_t count;
	
	int_accx -= accel[0][count];
	int_accy -= accel[1][count];
	int_accz -= accel[2][count];
	int_gyrox -= gyro[0][count];
	int_gyroy -= gyro[1][count];
	int_gyroz -= gyro[2][count];
	
	accel[0][count] = fliter_in->accx;
	accel[1][count] = fliter_in->accy;
	accel[2][count] = fliter_in->accz;
	gyro[0][count] = fliter_in->gyrox;
	gyro[1][count] = fliter_in->gyroy;
	gyro[2][count] = fliter_in->gyroz;

	int_accx += accel[0][count];
	int_accy += accel[1][count];
	int_accz += accel[2][count];
	int_gyrox += gyro[0][count];
	int_gyroy += gyro[1][count];
	int_gyroz += gyro[2][count];

	fliter_out->accx = int_accx/fliter_lenth;
	fliter_out->accy = int_accy/fliter_lenth;
	fliter_out->accz = int_accz/fliter_lenth;
	fliter_out->gyrox = int_gyrox/fliter_lenth;
	fliter_out->gyroy = int_gyroy/fliter_lenth;
	fliter_out->gyroz = int_gyroz/fliter_lenth;

	count ++;
	if(count >= fliter_lenth)	count = 0;
	
}
/**
  * @}
  */
void upload_data(int32_t px, int32_t py, int32_t vx, int32_t vy, short pitch, short roll, short yaw, short temp)
{
	uint8_t i;
	for (i = 0; i < 4; ++i)
		usart1_send_char(px >> (8*i));
	usart1_send_char('\t');
	for (i = 0; i < 4; ++i)
		usart1_send_char(py >> (8*i));
	usart1_send_char('\t');
	for (i = 0; i < 4; ++i)
		usart1_send_char(vx >> (8*i));
	usart1_send_char('\t');
	for (i = 0; i < 4; ++i)
		usart1_send_char(vy >> (8*i));
	usart1_send_char('\t');

	for (i = 0; i < 2; ++i)
		usart1_send_char(pitch >> (8*i));
	usart1_send_char('\t');
	for (i = 0; i < 2; ++i)
		usart1_send_char(roll >> (8*i));
	usart1_send_char('\t');
	for (i = 0; i < 2; ++i)
		usart1_send_char(yaw >> (8*i));
	usart1_send_char('\t');

	for (i = 0; i < 2; ++i)
		usart1_send_char(temp >> (8*i));
	usart1_send_char('\r');
	usart1_send_char('\n');
}

/**************** (C) COPYRIGHT STMicroelectronics *****END OF FILE*****************/
