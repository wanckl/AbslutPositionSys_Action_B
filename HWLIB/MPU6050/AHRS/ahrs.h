
#define R2A 		57.324841f				//弧度到角度
#define A2R    		0.0174533f				//角度到弧度
#define Acc_G 		0.0005981f				//加速度变成G
#define GYRO_PRE	0.06103515625f;

void get_euler_angle(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw);

