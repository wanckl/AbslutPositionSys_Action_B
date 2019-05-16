#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "inv_mpu.h"

//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Init(uint8_t addr)
{ 
	uint8_t ID = 0;
	
	MPU_Write_Byte(addr, MPU_PWR_MGMT1_REG, 0X80);	//��λMPU6050
	delay_ms(100);
	ID = MPU_Read_Byte(addr, MPU_DEVICE_ID_REG);
	printf("0x%x = 0x%x -- ", addr, ID);
	if(ID == 0x68)//����ID��ȷ
	{
		printf("MPU6050 @ 0x%x detected\n\n", addr);
		
		MPU_Write_Byte(addr, MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
		MPU_Set_Gyro_Fsr(addr, 3);						//�����Ǵ�����,��2000dps
		MPU_Set_Accel_Fsr(addr, 0);						//���ٶȴ�����,��2g
		MPU_Set_Rate(addr, 50);							//���ò�����50Hz
		MPU_Write_Byte(addr, MPU_INT_EN_REG,0X00);		//�ر������ж�
		MPU_Write_Byte(addr, MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
		MPU_Write_Byte(addr, MPU_FIFO_EN_REG,0X00);		//�ر�FIFO
		MPU_Write_Byte(addr, MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
		MPU_Write_Byte(addr, MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(addr, MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
	}
	else return 1;
	return 0;
}

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Gyro_Fsr(uint8_t addr, u8 fsr)
{
	return MPU_Write_Byte(addr, MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}

//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Accel_Fsr(uint8_t addr, uint8_t fsr)
{
	return MPU_Write_Byte(addr, MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_LPF(uint8_t addr, u16 lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(addr, MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Rate(uint8_t addr, u16 rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(addr, MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(addr, rate/2);						//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
float MPU_Get_Temperature(uint8_t addr)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(addr, MPU_TEMP_OUTH_REG, 2, buf); 
    raw=((u16)buf[0]<<8)|buf[1];
    temp=(((double) (raw + 13200)) / 280)-13;
    return temp;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Gyroscope(uint8_t addr, short *gx,short *gy,short *gz)
{
		uint8_t buf[6],res;  
		res=MPU_Read_Len(addr,MPU_GYRO_XOUTH_REG,6,buf);
		if(res==0)
		{
			*gx=((u16)buf[0]<<8)|buf[1];  
			*gy=((u16)buf[2]<<8)|buf[3];  
			*gz=((u16)buf[4]<<8)|buf[5];
		} 	
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Accelerometer(uint8_t addr, short *ax, short *ay, short *az)
{
    uint8_t buf[6],res;  
		res=MPU_Read_Len(addr,MPU_ACCEL_XOUTH_REG,6,buf);
		if(res==0)
		{
			*ax=((u16)buf[0]<<8)|buf[1];  
			*ay=((u16)buf[2]<<8)|buf[3];  
			*az=((u16)buf[4]<<8)|buf[5];
		} 	
    return res;;
}
//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	addr = mpu_addr;
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		if(IIC_Wait_Ack())		//�ȴ�ACK
		{
			IIC_Stop();	 
			return 1;		 
		}
	}
    IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	addr = mpu_addr;
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data) 				 
{
	addr = mpu_addr;
    IIC_Start(); 
	IIC_Send_Byte((addr << 1) | 0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(data);//��������
	if(IIC_Wait_Ack())	//�ȴ�ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t addr, uint8_t reg)
{
	uint8_t res;
	addr = mpu_addr;
    IIC_Start(); 
	IIC_Send_Byte((addr << 1)|0);//����������ַ+д����	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((addr << 1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	res=IIC_Read_Byte(0);//��ȡ����,����nACK 
    IIC_Stop();			//����һ��ֹͣ���� 
	return res;		
}

uint8_t action_dmp_init(uint8_t addr)
{
	mpu_addr = addr;
	return mpu_dmp_init();
}

uint8_t action_dmp_getdata(float *pitch,float *roll,float *yaw, imu_struct *mpustru)
{
	mpu_addr = mpustru->addr;
	mpustru->temp = MPU_Get_Temperature(mpu_addr);	//�õ��¶�ֵ
	return mpu_dmp_get_data(pitch, roll, yaw, mpustru);
}

uint8_t mpu_addr = 0xff;
