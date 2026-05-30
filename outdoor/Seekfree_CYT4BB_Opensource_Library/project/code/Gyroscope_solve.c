#include "Gyroscope_solve.h"
#include "zf_common_headfile.h"
#include "mymath.h"
/*你说得对，但是惯性测量单元（Inertial Measurement Unit）是一款由陀螺仪、
 * 加速度计和地磁计构成的运动测量模块。计算发生在一个被称作「姿态解算」的复杂算法，
 * 在这里，被原始数据选中的变量将被授予「四元数」，导引第四维之力。你将扮演一位名为「啥比大学生」的角色，
 * 在推导中邂逅性格迥异，能力独特的滤波算法。和他们一起对抗零飘，找回修正角度的同时，逐步发掘「欧拉角」的真相*/
//-------------------------------------------------------------------------------------------------------------------
// @brief       陀螺仪零漂初始化
// @param
// @return      void
// Sample usage:    通过采集一定数据求均值计算陀螺仪零点偏移值。后续 陀螺仪读取的数据 - 零飘值，即可去除零点偏移量。
//-------------------------------------------------------------------------------------------------------------------
icm_param_t Gyroscope_raw_data;         // 陀螺仪原始值
Gyroscope_param_t GyroOffset;                 // 陀螺仪零飘
Gyroscope_param_t Gyroscope_data;       //陀螺仪运算值
void Gyroscope_Offset_Init()
{
    uint16_t i;
    //short gx=0,gy=0,gz=0;
    long int gxx=0,gyy=0,gzz=0;
    GyroOffset.gyro_x = 0;
    GyroOffset.gyro_y = 0;
    GyroOffset.gyro_z = 0;
    for ( i = 0; i <= 2000; i++)
    {
        imu963ra_get_gyro();
        gxx+=imu963ra_gyro_x;
        gyy+=imu963ra_gyro_y;
        gzz+=imu963ra_gyro_z;
        // Data_Send(gx,gy,gz,GyroOffset.gyro_x,GyroOffset.gyro_y,GyroOffset.gyro_z,0,0);
        // delay_ms(5);
    }
    GyroOffset.gyro_x = (float)gxx/2000.0;
    GyroOffset.gyro_y = (float)gyy/2000.0;
    GyroOffset.gyro_z = (float)gzz/2000.0;
    // Data_Send(gx,gy,gz,GyroOffset.gyro_x,GyroOffset.gyro_y,GyroOffset.gyro_z,0,0);        delay_ms(5);
    // Data_Send(gx,gy,gz,GyroOffset.gyro_x,GyroOffset.gyro_y,GyroOffset.gyro_z,0,0);        delay_ms(5);
    // Data_Send(gx,gy,gz,GyroOffset.gyro_x,GyroOffset.gyro_y,GyroOffset.gyro_z,0,0);        delay_ms(5);
    Gyroscope_raw_data.acc_x = 0;
    Gyroscope_raw_data.acc_y = 0;
    Gyroscope_raw_data.acc_z = 0;
    Gyroscope_raw_data.gyro_x = 0;
    Gyroscope_raw_data.gyro_y = 0;
    Gyroscope_raw_data.gyro_z = 0;
    Gyroscope_data.acc_x = 0;
    Gyroscope_data.acc_y = 0;
    Gyroscope_data.acc_z = 0;
    Gyroscope_data.gyro_x = 0;
    Gyroscope_data.gyro_y = 0;
    Gyroscope_data.gyro_z = 0;
    init_arcsin_table();
}
// 对accel一阶低通滤波(参考匿名)，对gyro转成弧度每秒(2000dps)
//-------------------------------------------------------------------------------------------------------------------
// @brief       对获取的加速度进行低通滤波 对角速度进行单位换算
// @param
// @return      void
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Gyroscope_getValues()
{
    //低通滤波器系数 0~1
    float alpha = 1;            
    //获取六轴原始值
    //MPU_Get_Gyroscope(&Gyroscope_raw_data.gyro_x,&Gyroscope_raw_data.gyro_y,&Gyroscope_raw_data.gyro_z);
    //MPU_Get_Accelerometer(&Gyroscope_raw_data.acc_x,&Gyroscope_raw_data.acc_y,&Gyroscope_raw_data.acc_z);
    //一阶低通滤波，单位g
    imu963ra_get_gyro();
    imu963ra_get_acc();
    Gyroscope_raw_data.gyro_x=imu963ra_gyro_x;
    Gyroscope_raw_data.gyro_y=imu963ra_gyro_y;
    Gyroscope_raw_data.gyro_z=imu963ra_gyro_z;
    Gyroscope_raw_data.acc_x=imu963ra_acc_x;
    Gyroscope_raw_data.acc_y=imu963ra_acc_y;
    Gyroscope_raw_data.acc_z=imu963ra_acc_z;
    Gyroscope_data.acc_x = (((float) Gyroscope_raw_data.acc_x) * alpha) / 4096 + Gyroscope_data.acc_x * (1 - alpha);
    Gyroscope_data.acc_y = (((float) Gyroscope_raw_data.acc_y) * alpha) / 4096 + Gyroscope_data.acc_y * (1 - alpha);
    Gyroscope_data.acc_z = (((float) Gyroscope_raw_data.acc_z) * alpha) / 4096 + Gyroscope_data.acc_z * (1 - alpha);
    //陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
    Gyroscope_data.gyro_x = ((float)Gyroscope_raw_data.gyro_x - GyroOffset.gyro_x) * PI / 180 / 16.4f;
    Gyroscope_data.gyro_y = ((float)Gyroscope_raw_data.gyro_y - GyroOffset.gyro_y) * PI / 180 / 16.4f;
    Gyroscope_data.gyro_z = ((float)Gyroscope_raw_data.gyro_z - GyroOffset.gyro_z) * PI / 180 / 16.4f;
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       用互补滤波算法解算陀螺仪姿态(即利用加速度计修正陀螺仪的积分误差)
// @param
// @return      void
// Sample usage:    用互补滤波算法解算陀螺仪姿态(即利用加速度计修正陀螺仪的积分误差)因此使用姿态互补滤波，短期相信陀螺仪，长期相信加速度计。
//-------------------------------------------------------------------------------------------------------------------
#define delta_T     0.005f  // 采样周期5ms
float I_ex, I_ey, I_ez;  // 误差积分
quater_param_t Q_info = {1, 0, 0, 0};  // 四元数初始化
euler_param_t eulerAngle;              // 欧拉角
float icm_kp= 0.1;    // 加速度计的收敛速率比例增益//0.1
float icm_ki= 0.001;   // 陀螺仪收敛速率的积分增益//0.001
void Gyroscope_solve()
{
float halfT = 0.5 * delta_T;    // 采样周期一半
    float vx, vy, vz;               // 当前姿态计算得来的重力在三轴上的分量
    float ex, ey, ez;               // 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差
    float q0 = Q_info.q0;  //四元数
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;  //先相乘，方便后续计算
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    //float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    //float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float norm;
    // 正常静止状态为-g 反作用力。
    if(Gyroscope_data.acc_x * Gyroscope_data.acc_y * Gyroscope_data.acc_z == 0) // 加计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况
        return;
    // 对加速度数据进行归一化 得到单位加速度 (a^b -> 载体坐标系下的加速度)
    norm = myRsqrt(Gyroscope_data.acc_x * Gyroscope_data.acc_x + Gyroscope_data.acc_y * Gyroscope_data.acc_y + Gyroscope_data.acc_z * Gyroscope_data.acc_z);
    Gyroscope_data.acc_x = Gyroscope_data.acc_x * norm;
    Gyroscope_data.acc_y = Gyroscope_data.acc_y * norm;
    Gyroscope_data.acc_z = Gyroscope_data.acc_z * norm;
    // 载体坐标系下重力在三个轴上的分量
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    // g^b 与 a^b 做向量叉乘，得到陀螺仪的校正补偿向量e的系数
    ex = Gyroscope_data.acc_y * vz - Gyroscope_data.acc_z * vy;
    ey = Gyroscope_data.acc_z * vx - Gyroscope_data.acc_x * vz;
    ez = Gyroscope_data.acc_x * vy - Gyroscope_data.acc_y * vx;
    // 误差累加
    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;
    // 使用PI控制器消除向量积误差(陀螺仪漂移误差)
    Gyroscope_data.gyro_x = Gyroscope_data.gyro_x + icm_kp* ex + icm_ki* I_ex;
    Gyroscope_data.gyro_y = Gyroscope_data.gyro_y + icm_kp* ey + icm_ki* I_ey;
    Gyroscope_data.gyro_z = Gyroscope_data.gyro_z + icm_kp* ez + icm_ki* I_ez;
    // 一阶龙格库塔法求解四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为b系陀螺仪角速度。
    q0 = q0 + (-q1 * Gyroscope_data.gyro_x - q2 * Gyroscope_data.gyro_y - q3 * Gyroscope_data.gyro_z) * halfT;
    q1 = q1 + (q0 * Gyroscope_data.gyro_x + q2 * Gyroscope_data.gyro_z - q3 * Gyroscope_data.gyro_y) * halfT;
    q2 = q2 + (q0 * Gyroscope_data.gyro_y - q1 * Gyroscope_data.gyro_z + q3 * Gyroscope_data.gyro_x) * halfT;
    q3 = q3 + (q0 * Gyroscope_data.gyro_z + q1 * Gyroscope_data.gyro_y - q2 * Gyroscope_data.gyro_x) * halfT;
    // 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
    norm = myRsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;  // 用全局变量记录上一次计算的四元数值
    // 解算欧拉角
    q0 = Q_info.q0;
    q1 = Q_info.q1;
    q2 = Q_info.q2;
    q3 = Q_info.q3;
    // atan2返回输入坐标点与坐标原点连线与X轴正方形夹角的弧度值
    eulerAngle.pitch = -arcsin_lookup(2.f * q0 * q2 - 2.f * q1 * q3) * 180.f / PI;
    eulerAngle.roll = fast_atan2(2.f * q2 * q3 + 2.f * q0 * q1, -2.f * q1 * q1 - 2.f * q2 * q2 + 1) * 180.f / PI;
    eulerAngle.yaw = fast_atan2(2.f * q1 * q2 + 2.f * q0 * q3, -2.f * q2 * q2 - 2.f * q3 * q3 + 1) * 180.f / PI;
    if(eulerAngle.yaw<0)    //-90°变为270°    //从+-180°化为0-360°
         eulerAngle.yaw+=360;
    eulerAngle.yaw=360-eulerAngle.yaw;//变为顺时针增加
}
#define Q_angle     0.001 //角度噪声
#define Q_gyro      0.003//漂移噪声
#define R_angle     0.5  //角度测量噪声值
float dt=0.005;
float Kal_Gyro;
float gyro;
int16_t mmax, mmaz;
float angle, angleSpeed;
float Angle_Kalman;
int16_t Pre_Angle_Kalman;
float angleSpeedIntegral;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
char  C_0 = 1;
/* *******
 * @brief  卡尔曼滤波, 输入带有噪声的参数组, 返回可靠的参数组, 此处用于获得变化较稳定的角度值
 * @param[in]  Accel 由加速度计得到的角度参量
 * \param[in]  Gyro 由陀螺仪得到的角速度参量
 * @retval 稳定可靠的角度值, 用于直立控制
 *      eulerAngle.roll=KalmanFilter(fast_atan2( Gyroscope_data.acc_x*4, Gyroscope_data.acc_z*4)*180/3.14159,Gyroscope_data.gyro_y/16.4);
 */     
float KalmanFilter(float Accel,float Gyro)
{
    static float Angle = 0;//, Gyro_y = 0;
    Angle+=(Gyro - Q_bias) * dt; //先验估计
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3]= Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;
    Angle_err = Accel - Angle;  //zk-先验估计
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];
    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;
    Angle   += K_0 * Angle_err;  //后验估计
    Q_bias  += K_1 * Angle_err;  //后验估计
    //Gyro_y   = Gyro - Q_bias;    //输出值(后验估计)的微分=角速度
    return Angle;
}
