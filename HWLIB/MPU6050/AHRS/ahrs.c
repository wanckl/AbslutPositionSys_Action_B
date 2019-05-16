#include "sys.h"
#include "math.h"
#include "ahrs.h"
#include "mpu6050.h"

/*************************************************
���ƣ�float invSqrt(float x) 
���ܣ����ټ���ƽ��������
�������������ֵ
���������
����ֵ��ƽ��������
**************************************************/
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y *= (1.5f - (halfx * y * y));
  return y;
}

#define Kp 1.2f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0f                        // integral gain governs rate of convergence of gyroscope biases
const float halfT = 0.005f;             // half the sample period�������ڵ�һ��
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // quaternion elements representing the estimated orientation
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;    	// scaled integral error

/*************************************************
���ƣ�get_euler_angle(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw)
���ܣ���ȡŷ����
���������3�������� 3����ٶ�
���������ŷ����
����ֵ��
**************************************************/
void get_euler_angle(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw)
{
	float recipNorm;
	float vx, vy, vz;
	float ex, ey, ez;
	
	gx *= GYRO_PRE;
    gy *= GYRO_PRE;
    gz *= GYRO_PRE;

	if(ax*ay*az==0)
 		return;
		
	gx *= A2R;
    gy *= A2R;
    gz *= A2R;
	
	// Normalise accelerometer measurement
	// ��һ��
	recipNorm = invSqrt(ax*ax + ay*ay + az*az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// Estimated direction of gravity and vector perpendicular to magnetic flux
    // ���ݵ�ǰ��Ԫ����ֵ̬���Ƹ���������  ���ںͼ��ٶȼƲ����������������Ա�
	vx = q1*q3 - q0*q2;
	vy = q0*q1 + q2*q3;
	vz = q0*q0 - 0.5f + q3*q3;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	// ʹ�ò������������������ʵ�ʲ�����������������������֮������
	ex = (ay*vz - az*vy);           //�������������õ���־������
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	exInt += ex * Ki;				//�������л���
	eyInt += ey * Ki;
	ezInt += ez * Ki;

	// adjusted gyroscope measurements
	gx += Kp*ex + exInt;			//�����PI�󲹳��������ǣ����������Ư��
	gy += Kp*ey + eyInt;
	gz += Kp*ez + ezInt;			//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�

	// integrate quaternion rate and normalise			//��Ԫ����΢�ַ���
	q0 += (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 += (q0*gx + q2*gz - q3*gy)*halfT;
	q2 += (q0*gy - q1*gz + q3*gx)*halfT;
	q3 += (q0*gz + q1*gy - q2*gx)*halfT;

	// normalise quaternion
	recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	*roll  =  asin(-2 * q1 * q3 + 2 * q0 * q2) * R2A;  
	*pitch =  atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * R2A; 
	*yaw   = -atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 -2*q3*q3 + 1) * R2A; 

}
