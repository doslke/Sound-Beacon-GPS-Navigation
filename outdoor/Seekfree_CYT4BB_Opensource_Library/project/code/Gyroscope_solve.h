#ifndef GYROSCOPE_SOLVE_H_
#define GYROSCOPE_SOLVE_H_
#include "zf_common_headfile.h"
#include "mymath.h"
typedef struct
{
    short acc_x;
    short acc_y;
    short acc_z;
    short gyro_x;
    short gyro_y;
    short gyro_z;
}icm_param_t;   //陀螺仪的原始值
typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}Gyroscope_param_t;   //陀螺仪的原始值
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
}quater_param_t; // 四元数
typedef struct
{
    float pitch;
    float roll;
    double yaw;
}euler_param_t; // 欧拉角
extern quater_param_t Q_info;
extern euler_param_t eulerAngle;              	// 欧拉角
extern icm_param_t Gyroscope_raw_data;         	// 陀螺仪原始值
extern Gyroscope_param_t GyroOffset;            // 陀螺仪零飘
extern Gyroscope_param_t Gyroscope_data;       	// 陀螺仪运算值

void Gyroscope_getValues(void);
void Gyroscope_solve(void);
void Gyroscope_Offset_Init(void);
float KalmanFilter(float Accel,float Gyro);
#endif /* GYROSCOPE_SOLVE_H_ */
