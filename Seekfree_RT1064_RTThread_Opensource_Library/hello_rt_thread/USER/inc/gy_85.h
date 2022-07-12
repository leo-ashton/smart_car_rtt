#ifndef __GY_85_H
#define __GY_85_H

#include "common.h"

#define SENSORS_GRAVITY_EARTH (9.80665F)

//定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define ADXL345_Addr 0x53  //加速度传感器器件地址
#define ITG3205_Addr 0x68  //陀螺仪传感器器件地址
#define HMC5883L_Addr 0x1E //磁场传感器器件地址

//定义HMC5883L配置寄存器地址
#define HMC5883l_CONFIG_A 0x00
#define HMC5883l_CONFIG_B 0x01
#define HMC5883l_MODECONFIG 0x02

//定义ITG3205内部地址********************
#define WHO 0x00
#define SMPL 0x15
#define DLPF 0x16
#define INT_C 0x17
#define INT_S 0x1A
#define TMP_H 0x1B
#define TMP_L 0x1C
#define PWR_M 0x3E

typedef struct INT16_XYZ
{
	int16 X;
	int16 Y;
	int16 Z;
} INT16_XYZ;

typedef struct float_XYZ
{
	float X;
	float Y;
	float Z;
} float_XYZ;

typedef struct KFP
{
	float LastP; //上次估算协方差 初始化值为0.02
	float Now_P; //当前估算协方差 初始化值为0
	float out;	 //卡尔曼滤波器输出 初始化值为0
	float Kg;	 //卡尔曼增益 初始化值为0
	float Q;	 //过程噪声协方差 初始化值为0.001
	float R;	 //观测噪声协方差 初始化值为0.543
} KFP;			 // Kalman Filter parameter

//*********************相关函数声明
void read_ADXL345(void);
void Init_ADXL345(void);
float read_hmc5883l(void);
void Init_HMC5883L(void);
void Init_ITG3205(void);
void read_ITG3205(void);
void Dis_get(void);
void V_get(void);
void V2_get();
// float kalmanFilter(float in_Data, int i);
float kalmanFilter(KFP *kfp, float input);
void ITG3205_Offset_Calc(void);
void Angle_get(void);
#endif