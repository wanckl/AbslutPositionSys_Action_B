#include "sys.h"
#include "math.h"
#include "ahrs.h"
#include "mpu6050.h"

/*************************************************
名称：float invSqrt(float x) 
功能：快速计算平方根倒数
输入参数：输入值
输出参数：
返回值：平方根倒数
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
const float halfT = 0.005f;             // half the sample period采样周期的一半
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // quaternion elements representing the estimated orientation
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;    	// scaled integral error

/*************************************************
名称：get_euler_angle(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw)
功能：获取欧拉角
输入参数：3轴陀螺仪 3轴加速度
输出参数：欧拉角
返回值：
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
	// 归一化
	recipNorm = invSqrt(ax*ax + ay*ay + az*az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// Estimated direction of gravity and vector perpendicular to magnetic flux
    // 根据当前四元数姿态值估计各重力分量  用于和加速度计测量的重力分量做对比
	vx = q1*q3 - q0*q2;
	vy = q0*q1 + q2*q3;
	vz = q0*q0 - 0.5f + q3*q3;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	// 使用叉积来计算估算的重力和实际测量的重力这两个重力向量之间的误差
	ex = (ay*vz - az*vy);           //向量外积在相减得到差分就是误差
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	exInt += ex * Ki;				//对误差进行积分
	eyInt += ey * Ki;
	ezInt += ez * Ki;

	// adjusted gyroscope measurements
	gx += Kp*ex + exInt;			//将误差PI后补偿到陀螺仪，即补偿零点漂移
	gy += Kp*ey + eyInt;
	gz += Kp*ez + ezInt;			//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

	// integrate quaternion rate and normalise			//四元数的微分方程
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
