#include "mpu6050.h"
#include "mpu9250.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"


void MPU9250_GetMag(uint8_t addr, short *mag_x, short *mag_y, short *mag_z)
{
	uint8_t BUF[6];
	MPU_Write_Byte(addr, 0x37, 0x02);	//Switch MPU6050 Bypass Mode 
	delay_ms(10);
	MPU_Write_Byte(addr, 0x0A, 0x01);
	delay_ms(10);
	BUF[0]=MPU_Read_Byte(addr,MAG_XOUT_L);
	BUF[1]=MPU_Read_Byte(addr,MAG_XOUT_H);
	*mag_x=(BUF[1]<<8)|BUF[0];

	BUF[2]=MPU_Read_Byte(addr,MAG_YOUT_L);
	BUF[3]=MPU_Read_Byte(addr,MAG_YOUT_H);
	*mag_y=	(BUF[3]<<8)|BUF[2];	   //读取计算Y轴数据
	 
	BUF[4]=MPU_Read_Byte(addr,MAG_ZOUT_L);
	BUF[5]=MPU_Read_Byte(addr,MAG_ZOUT_H);
	*mag_z=	(BUF[5]<<8)|BUF[4];	   //读取计算Z轴数据
}
